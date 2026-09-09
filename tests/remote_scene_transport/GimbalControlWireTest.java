package com.questnewton;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.KeyStore;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLSocket;
import static com.questnewton.RemoteSceneWireTest.*;
public final class GimbalControlWireTest {
    static byte[] authControl(SSLSocket peer) throws Exception {
        byte[] b=peer.getInputStream().readNBytes(36);
        check(b.length==36&&b[0]=='Q'&&b[1]=='G'&&b[2]=='C'&&b[3]=='1',"control channel auth magic must be separate");return Arrays.copyOfRange(b,4,36);
    }
    public static void main(String[] args) throws Exception {
        char[] password=System.getenv("REMOTE_TEST_STORE_PASSWORD").toCharArray();KeyStore store=KeyStore.getInstance("PKCS12");
        try(InputStream in=Files.newInputStream(Path.of(args[0]))){store.load(in,password);}
        KeyManagerFactory keys=KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());keys.init(store,password);
        SSLContext context=SSLContext.getInstance("TLS");context.init(keys.getKeyManagers(),null,null);
        byte[] pin=MessageDigest.getInstance("SHA-256").digest(store.getCertificate("remote-fixture").getEncoded());
        byte[] token=new byte[32];new SecureRandom().nextBytes(token);byte[] greeting=new byte[96],reply=new byte[96],command=new byte[64];
        Arrays.fill(greeting,(byte)17);Arrays.fill(reply,(byte)19);Arrays.fill(command,(byte)23);
        try(Server server=new Server(context,peer->{check(MessageDigest.isEqual(authControl(peer),token),"control token mismatch");
            peer.getOutputStream().write(greeting);peer.getOutputStream().flush();check(Arrays.equals(peer.getInputStream().readNBytes(64),command),"64-byte control command changed");
            peer.getOutputStream().write(reply);peer.getOutputStream().flush();});RemoteSceneWire wire=new RemoteSceneWire()){
            wire.connect(server.client(pin,token),"QGC1");check(Arrays.equals(wire.readControl(null),greeting),"initial fixed feedback mismatch");
            rejects(()->wire.readControl(new byte[63]));check(Arrays.equals(wire.readControl(command),reply),"fixed command reply mismatch");rejects(()->wire.readControl(null));
        }
        CountDownLatch stop=new CountDownLatch(1);
        try(Server server=new Server(context,peer->{authControl(peer);peer.getOutputStream().write(greeting);peer.getOutputStream().flush();peer.getInputStream().readNBytes(64);
            for(byte b:reply){peer.getOutputStream().write(b);peer.getOutputStream().flush();if(stop.await(60,TimeUnit.MILLISECONDS))break;}});RemoteSceneWire wire=new RemoteSceneWire()){
            wire.connect(server.client(pin,token),"QGC1");wire.readControl(null);long start=System.nanoTime();rejects(()->wire.readControl(command));stop.countDown();
            check(System.nanoTime()-start<TimeUnit.MILLISECONDS.toNanos(1500),"trickled control feedback bypassed RPC deadline");
        }
        CountDownLatch requested=new CountDownLatch(1),release=new CountDownLatch(1);
        try(Server server=new Server(context,peer->{authControl(peer);peer.getOutputStream().write(greeting);peer.getOutputStream().flush();peer.getInputStream().readNBytes(64);requested.countDown();release.await(2,TimeUnit.SECONDS);});RemoteSceneWire wire=new RemoteSceneWire()){
            wire.connect(server.client(pin,token),"QGC1");wire.readControl(null);AtomicReference<Throwable> outcome=new AtomicReference<>();
            Thread worker=new Thread(()->{try{wire.readControl(command);outcome.set(new AssertionError("unexpected reply"));}catch(Throwable e){outcome.set(e);}});worker.start();
            check(requested.await(1,TimeUnit.SECONDS),"control command not delivered");rejects(()->wire.readControl(command));wire.close();worker.join(1000);release.countDown();
            check(!worker.isAlive()&&outcome.get() instanceof java.io.IOException,"control close did not interrupt pending RPC");
        }
        try(RemoteSceneWire wire=new RemoteSceneWire()){rejects(()->wire.readControl(null));rejects(()->wire.connect(RemoteSceneWire.Config.parse(config("127.0.0.1",1,pin,token)),"QRSX"));}
        Path privateDir=Path.of(args[0]).getParent().resolve("gimbal-private");Files.createDirectories(privateDir);int[] fileReads={0};
        android.content.Context app=new android.content.Context(){
            @Override public java.io.File getFilesDir(){fileReads[0]++;return privateDir.toFile();}
            @Override public android.content.res.AssetManager getAssets(){return null;}
        };
        GimbalControlConnection cancelled=new GimbalControlConnection(app);check(fileReads[0]==0,"camera constructor performed config I/O");cancelled.close();rejects(cancelled::open);
        check(fileReads[0]==0,"cancelled camera open read private config");
        try(Server server=new Server(context,peer->{check(MessageDigest.isEqual(authControl(peer),token),"Android adapter used wrong credentials");peer.getOutputStream().write(greeting);peer.getOutputStream().flush();});
            GimbalControlConnection adapter=new GimbalControlConnection(app)){
            Files.writeString(privateDir.resolve("remote-gimbal-client.json"),config(server.socket.getInetAddress().getHostAddress(),server.socket.getLocalPort(),pin,token));
            check(Arrays.equals(adapter.open(),greeting),"Android camera adapter greeting changed");
        }
        System.out.println("control TLS auth, fixed framing, deadlines, single pending request and close passed");
    }
}
