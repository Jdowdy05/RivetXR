package com.questnewton;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.KeyStore;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.zip.DeflaterOutputStream;
import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLServerSocket;
import javax.net.ssl.SSLSocket;

public final class RemoteSceneWireTest {
    interface Checked { void run() throws Exception; }
    interface Serve { void run(SSLSocket socket) throws Exception; }
    static void check(boolean value,String reason) { if (!value) throw new AssertionError(reason); }
    static void rejects(Checked action) throws Exception {
        try { action.run(); } catch (IOException expected) { return; }
        throw new AssertionError("Invalid wire/config input was accepted");
    }
    static String hex(byte[] bytes) {
        StringBuilder out=new StringBuilder();for(byte value:bytes)out.append(String.format("%02x",value&255));return out.toString();
    }
    static String config(String host,int port,byte[] pin,byte[] token) {
        return "{\"version\":1,\"host\":\""+host+"\",\"port\":"+port+
            ",\"certificate_sha256\":\""+hex(pin)+"\",\"token_hex\":\""+hex(token)+"\"}";
    }
    static byte[] compressed(byte[] raw) throws Exception {
        ByteArrayOutputStream bytes=new ByteArrayOutputStream();
        try(DeflaterOutputStream out=new DeflaterOutputStream(bytes)){out.write(raw);}return bytes.toByteArray();
    }
    static byte[] frame(byte[] zipped,int rawLength) throws Exception {
        ByteArrayOutputStream bytes=new ByteArrayOutputStream();DataOutputStream out=new DataOutputStream(bytes);
        out.writeInt(rawLength);out.writeInt(zipped.length);out.write(zipped);return bytes.toByteArray();
    }
    static void unit() throws Exception {
        byte[] pin=new byte[32],token=new byte[32];Arrays.fill(token,(byte)7);
        String good=config("127.0.0.1",12345,pin,token);
        check(RemoteSceneWire.Config.parse(good).address.getAddress().length==4,"numeric IPv4");
        check(RemoteSceneWire.Config.parse(config("::1",12345,pin,token)).address.getAddress().length==16,"numeric IPv6");
        check(RemoteSceneWire.Config.parse(config("2001:db8::192.0.2.1",12345,pin,token)).address.getAddress().length==16,"IPv6 with numeric tail");
        for(String host:new String[]{"localhost","example.com","127.1","01.2.3.4","256.0.0.1","1:2:3","fe80::1%wlan0","1::2::3"})
            rejects(()->RemoteSceneWire.Config.parse(config(host,12345,pin,token)));
        for(String invalid:new String[]{good.replace("\"version\":1","\"version\":1,\"version\":1"),
            good.replace("\"version\":1","\"version\":1.0"),good.replace("\"port\":12345","\"port\":0"),
            good.replace("\"port\":12345","\"port\":65536"),good.replace("\"host\"","\"unknown\""),
            good+"x",good.replace(hex(pin),"z"+hex(pin).substring(1))})rejects(()->RemoteSceneWire.Config.parse(invalid));
        rejects(()->RemoteSceneWire.Config.parse(" ".repeat(4097)));
        byte[] raw=new byte[128];raw[0]='R';raw[1]='S';raw[2]='C';raw[3]='N';
        byte[] zipped=compressed(raw),packet=frame(zipped,raw.length);
        check(Arrays.equals(raw,RemoteSceneWire.readFrame(new ByteArrayInputStream(packet))),"wire round trip");
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(Arrays.copyOf(packet,packet.length-1))));
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(zipped,127))));
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(zipped,1024*1024+1))));
        byte[] oversized=Arrays.copyOf(packet,8);oversized[4]=(byte)255;
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(oversized)));
        byte[] extra=Arrays.copyOf(zipped,zipped.length+1);
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(extra,raw.length))));
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(zipped,raw.length+1))));
        byte[] bomb=compressed(new byte[4096]);
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(bomb,128))));
        rejects(()->RemoteSceneWire.readFrame(new ByteArrayInputStream(frame(new byte[]{1,2,3},128))));
        rejects(()->RemoteSceneWire.readRaw(new ByteArrayInputStream(new byte[RemoteSceneWire.MAX_BYTES+1])));
        check(RemoteSceneWire.readRaw(new ByteArrayInputStream(new byte[1024*1024+1])).length==1024*1024+1,"v3 transport budget must exceed old1MiB cap");
        check(Arrays.equals(raw,RemoteSceneWire.readRaw(new ByteArrayInputStream(raw))),"bounded demo bytes");
        RemoteSceneWire closed=new RemoteSceneWire();closed.close();closed.close();
        rejects(()->closed.connect(RemoteSceneWire.Config.parse(good)));
        System.out.println("wire/config/decompression bounds passed");
    }
    static final class Server implements AutoCloseable {
        final SSLServerSocket socket;final Thread thread;final AtomicReference<Throwable> failure=new AtomicReference<>();
        Server(SSLContext context,Serve action) throws Exception {
            socket=(SSLServerSocket)context.getServerSocketFactory().createServerSocket(0,1,java.net.InetAddress.getLoopbackAddress());
            socket.setSoTimeout(5000);socket.setEnabledProtocols(new String[]{"TLSv1.2"});
            thread=new Thread(()->{try(SSLSocket peer=(SSLSocket)socket.accept()){
                peer.setSoTimeout(2500);peer.startHandshake();action.run(peer);
            }catch(Throwable error){failure.set(error);}},"LoopbackRemoteSceneFixture");thread.setDaemon(true);thread.start();
        }
        RemoteSceneWire.Config client(byte[] pin,byte[] token) throws Exception {
            return RemoteSceneWire.Config.parse(config(socket.getInetAddress().getHostAddress(),socket.getLocalPort(),pin,token));
        }
        public void close() throws Exception {socket.close();thread.join(3500);check(!thread.isAlive(),"fixture worker did not stop");}
    }
    static byte[] auth(SSLSocket peer) throws Exception {
        byte[] bytes=peer.getInputStream().readNBytes(36);
        check(bytes.length==36&&bytes[0]=='Q'&&bytes[1]=='R'&&bytes[2]=='S'&&bytes[3]=='1',"TLS auth framing");
        return Arrays.copyOfRange(bytes,4,36);
    }
    static void tls(Path storePath) throws Exception {
        char[] password=System.getenv("REMOTE_TEST_STORE_PASSWORD").toCharArray();
        KeyStore store=KeyStore.getInstance("PKCS12");try(InputStream in=Files.newInputStream(storePath)){store.load(in,password);}
        KeyManagerFactory keys=KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());keys.init(store,password);
        SSLContext serverContext=SSLContext.getInstance("TLS");serverContext.init(keys.getKeyManagers(),null,null);
        byte[] pin=MessageDigest.getInstance("SHA-256").digest(store.getCertificate("remote-fixture").getEncoded());
        byte[] token=new byte[32];new SecureRandom().nextBytes(token);byte[] raw=new byte[128];Arrays.fill(raw,(byte)42);
        byte[] packet=frame(compressed(raw),raw.length);
        try(Server server=new Server(serverContext,peer->{check(MessageDigest.isEqual(auth(peer),token),"token delivery");peer.getOutputStream().write(packet);peer.getOutputStream().flush();});
            RemoteSceneWire client=new RemoteSceneWire()) {
            client.connect(server.client(pin,token));check(Arrays.equals(client.readFrame(),raw),"TLS payload");
        }
        byte[] wrong=pin.clone();wrong[0]^=1;AtomicInteger exposed=new AtomicInteger();
        try(Server server=new Server(serverContext,peer->{int value=peer.getInputStream().read();if(value>=0)exposed.incrementAndGet();});
            RemoteSceneWire client=new RemoteSceneWire()) {
            rejects(()->client.connect(server.client(wrong,token)));
        }
        check(exposed.get()==0,"token was sent before certificate pin validation");
        byte[] badToken=token.clone();badToken[0]^=1;AtomicReference<byte[]> observedToken=new AtomicReference<>();
        try(Server server=new Server(serverContext,peer->{observedToken.set(auth(peer));if(!MessageDigest.isEqual(observedToken.get(),token))return;throw new AssertionError("bad token accepted");});
            RemoteSceneWire client=new RemoteSceneWire()) {
            client.connect(server.client(pin,badToken));rejects(client::readFrame);
        }
        check(Arrays.equals(observedToken.get(),badToken),"client did not send the configured token bytes");
        CountDownLatch authenticated=new CountDownLatch(1),release=new CountDownLatch(1);
        try(Server server=new Server(serverContext,peer->{auth(peer);authenticated.countDown();release.await(3,TimeUnit.SECONDS);});
            RemoteSceneWire client=new RemoteSceneWire()) {
            client.connect(server.client(pin,token));check(authenticated.await(2,TimeUnit.SECONDS),"fixture not authenticated");
            AtomicReference<Throwable> outcome=new AtomicReference<>();CountDownLatch reading=new CountDownLatch(1);
            Thread reader=new Thread(()->{reading.countDown();try{client.readFrame();outcome.set(new AssertionError("unexpected packet"));}catch(Throwable error){outcome.set(error);}});
            reader.start();reading.await();long start=System.nanoTime();client.close();reader.join(1500);release.countDown();
            check(!reader.isAlive()&&outcome.get() instanceof IOException,"close did not interrupt blocked receive");
            check(System.nanoTime()-start<TimeUnit.MILLISECONDS.toNanos(1500),"receive close exceeded shutdown bound");
        }
        try(java.net.ServerSocket stalled=new java.net.ServerSocket(0,1,java.net.InetAddress.getLoopbackAddress());
            RemoteSceneWire client=new RemoteSceneWire()) {
            CountDownLatch accepted=new CountDownLatch(1),done=new CountDownLatch(1);
            Thread peer=new Thread(()->{try(java.net.Socket ignored=stalled.accept()){accepted.countDown();done.await(3,TimeUnit.SECONDS);}catch(Exception ignored){}});peer.start();
            AtomicReference<Throwable> result=new AtomicReference<>();
            RemoteSceneWire.Config cfg=RemoteSceneWire.Config.parse(config(stalled.getInetAddress().getHostAddress(),stalled.getLocalPort(),pin,token));
            Thread connector=new Thread(()->{try{client.connect(cfg);}catch(Throwable error){result.set(error);}});connector.start();
            check(accepted.await(2,TimeUnit.SECONDS),"connect fixture was not reached");client.close();connector.join(1500);done.countDown();peer.join(1500);
            check(!connector.isAlive()&&result.get() instanceof IOException,"close did not interrupt connect/handshake");
        }
        CountDownLatch stopTrickle=new CountDownLatch(1);
        try(Server server=new Server(serverContext,peer->{auth(peer);byte[] encoded=compressed(raw);
            DataOutputStream out=new DataOutputStream(peer.getOutputStream());out.writeInt(raw.length);out.writeInt(encoded.length);out.flush();
            for(byte value:encoded){out.writeByte(value);out.flush();if(stopTrickle.await(400,TimeUnit.MILLISECONDS))break;}});
            RemoteSceneWire client=new RemoteSceneWire()) {
            client.connect(server.client(pin,token));long start=System.nanoTime();boolean expired=false;
            try{client.readFrame();}catch(java.net.SocketTimeoutException expected){expired=true;}finally{stopTrickle.countDown();}
            check(expired&&System.nanoTime()-start<TimeUnit.SECONDS.toNanos(5),"trickled frame bypassed whole-frame deadline");
        }
        CountDownLatch stopHandshake=new CountDownLatch(1);
        try(java.net.ServerSocket stalled=new java.net.ServerSocket(0,1,java.net.InetAddress.getLoopbackAddress());
            RemoteSceneWire client=new RemoteSceneWire()) {
            Thread peer=new Thread(()->{try(java.net.Socket socket=stalled.accept()){
                // Incomplete but framed TLS handshake record: bytes keep arriving
                // faster than the idle timeout, exercising the absolute deadline.
                socket.getOutputStream().write(new byte[]{22,3,3,1,0});socket.getOutputStream().flush();
                for(int i=0;i<256;i++){socket.getOutputStream().write(0);socket.getOutputStream().flush();if(stopHandshake.await(100,TimeUnit.MILLISECONDS))break;}
            }catch(Exception ignored){}});peer.setDaemon(true);peer.start();
            long start=System.nanoTime();boolean expired=false;
            try{client.connect(RemoteSceneWire.Config.parse(config(stalled.getInetAddress().getHostAddress(),stalled.getLocalPort(),pin,token)));}
            catch(java.net.SocketTimeoutException expected){expired=true;}finally{stopHandshake.countDown();peer.join(1500);}
            check(expired&&System.nanoTime()-start<TimeUnit.SECONDS.toNanos(6),"trickled handshake bypassed absolute connect deadline");
        }
        System.out.println("loopback TLS pin/auth/receive-close/connect-close/deadlines passed");
    }
    public static void main(String[] args) throws Exception {
        if(args.length==3 && args[0].equals("--publisher")){
            RemoteSceneWire.Config cfg=RemoteSceneWire.Config.parse(Files.readString(Path.of(args[1])));
            try(RemoteSceneWire client=new RemoteSceneWire()){
                client.connect(cfg);Files.write(Path.of(args[2]),client.readFrame());
            }
            System.out.println("Python publisher interoperability passed");return;
        }
        unit();if(args.length==1)tls(Path.of(args[0]));
    }
}
