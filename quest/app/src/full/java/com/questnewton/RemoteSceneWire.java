package com.questnewton;

import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.security.GeneralSecurityException;
import java.security.MessageDigest;
import java.security.cert.CertificateException;
import java.security.cert.X509Certificate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLSocket;
import javax.net.ssl.SSLSocketFactory;
import javax.net.ssl.TrustManager;
import javax.net.ssl.X509TrustManager;

/** Pure-Java bounded TLS/framing helper. No Android, DNS, physics or rendering. */
public final class RemoteSceneWire implements AutoCloseable {
    public static final int MAX_BYTES=4*1024*1024;
    public static final int MAX_CONFIG_BYTES=4096;
    private static final int CONNECT_MS=2500,CONNECT_TOTAL_MS=4000,READ_IDLE_MS=1000,FRAME_MS=3000;
    private final Object gate=new Object();
    private Socket tcp;
    private SSLSocket tls;
    private InputStream input;
    private OutputStream output;
    private ScheduledExecutorService deadline;
    private ScheduledExecutorService rpcDeadline;
    private boolean controlMode,controlStarted,rpcPending;
    private boolean started,closed;
    private volatile boolean connectExpired;

    public static final class Config {
        public final String host;
        public final InetAddress address;
        public final int port;
        private final byte[] pin,token;
        private Config(String host,int port,byte[] pin,byte[] token) throws IOException {
            this.host=host;this.address=literalAddress(host);this.port=port;this.pin=pin.clone();this.token=token.clone();
        }
        public static Config parse(String json) throws IOException {
            if(json==null || json.length()>MAX_CONFIG_BYTES)throw new IOException("Remote scene configuration is too large");
            Map<String,Object> fields=new FlatJson(json).parse();
            if(fields.size()!=5 || !Long.valueOf(1).equals(fields.get("version")) || !(fields.get("host") instanceof String) ||
               !(fields.get("port") instanceof Long) || !(fields.get("certificate_sha256") instanceof String) ||
               !(fields.get("token_hex") instanceof String))throw new IOException("Invalid remote scene configuration fields");
            long port=(Long)fields.get("port");
            if(port<1 || port>65535)throw new IOException("Invalid remote scene port");
            return new Config((String)fields.get("host"),(int)port,hex32((String)fields.get("certificate_sha256")),hex32((String)fields.get("token_hex")));
        }
    }

    public void connect(Config config) throws IOException {connect(config,"QRS1");}
    public void connect(Config config,String protocol) throws IOException {
        if(!"QRS1".equals(protocol)&&!"QGC1".equals(protocol))throw new IOException("Unknown remote authentication protocol");
        final Socket transport=new Socket();
        final ScheduledExecutorService timer=Executors.newSingleThreadScheduledExecutor(task->{
            Thread thread=new Thread(task,"RemoteSceneConnectDeadline");thread.setDaemon(true);return thread;
        });
        synchronized(gate){
            if(closed || started){timer.shutdownNow();transport.close();throw new IOException("Remote scene connection is closed or already started");}
            started=true;tcp=transport;deadline=timer;controlMode=protocol.equals("QGC1");
        }
        try {
            timer.schedule(()->{connectExpired=true;close();},CONNECT_TOTAL_MS,TimeUnit.MILLISECONDS);
            // Provider initialization can be slow on a cold JVM. Finish local
            // setup before opening TCP, so it does not consume the server's
            // pre-authentication timeout. The total client deadline still runs.
            SSLContext context=SSLContext.getInstance("TLS");
            final byte[] pin=config.pin.clone();
            context.init(null,new TrustManager[]{new X509TrustManager(){
                @Override public X509Certificate[] getAcceptedIssuers(){return new X509Certificate[0];}
                @Override public void checkClientTrusted(X509Certificate[] chain,String authType) throws CertificateException {
                    throw new CertificateException("Client certificate trust is unsupported");
                }
                @Override public void checkServerTrusted(X509Certificate[] chain,String authType) throws CertificateException {
                    if(chain==null || chain.length==0)throw new CertificateException("Remote scene server certificate is missing");
                    chain[0].checkValidity();
                    try {
                        byte[] actual=MessageDigest.getInstance("SHA-256").digest(chain[0].getEncoded());
                        if(!MessageDigest.isEqual(pin,actual))throw new CertificateException("Remote scene certificate pin mismatch");
                    }catch(java.security.NoSuchAlgorithmException error){throw new CertificateException("SHA-256 is unavailable",error);}
                }
            }},null);
            SSLSocketFactory factory=context.getSocketFactory();
            transport.setSoTimeout(READ_IDLE_MS);transport.setTcpNoDelay(true);
            transport.connect(new InetSocketAddress(config.address,config.port),CONNECT_MS);
            SSLSocket secure=(SSLSocket)factory.createSocket(transport,config.address.getHostAddress(),config.port,true);
            synchronized(gate){if(closed){secure.close();throw new IOException("Remote scene connection is closed");}tls=secure;}
            List<String> protocols=new ArrayList<>();
            for(String candidate:secure.getSupportedProtocols())if(candidate.equals("TLSv1.2") || candidate.equals("TLSv1.3"))protocols.add(candidate);
            if(protocols.isEmpty())throw new IOException("TLS 1.2 or newer is unavailable");
            secure.setEnabledProtocols(protocols.toArray(new String[0]));secure.setUseClientMode(true);secure.setSoTimeout(READ_IDLE_MS);
            secure.startHandshake(); // The trust manager checks the exact DER leaf pin before any token write.
            byte[] request=new byte[36];for(int i=0;i<4;i++)request[i]=(byte)protocol.charAt(i);
            System.arraycopy(config.token,0,request,4,32);
            try {secure.getOutputStream().write(request);secure.getOutputStream().flush();}finally{Arrays.fill(request,(byte)0);}
            synchronized(gate){if(closed)throw new IOException("Remote scene connection is closed");input=secure.getInputStream();output=secure.getOutputStream();}
        }catch(GeneralSecurityException error){close();throw new IOException("Remote scene TLS initialization failed",error);
        }catch(IOException error){close();if(connectExpired)throw new SocketTimeoutException("Remote scene connect/handshake deadline exceeded");throw error;
        }catch(RuntimeException error){close();throw new IOException("Remote scene connection initialization failed",error);
        }finally {timer.shutdownNow();synchronized(gate){if(deadline==timer)deadline=null;}}
    }
    /** QGC1 only: initial null request reads96 bytes; subsequent requests are64 bytes.
     * A scheduled close bounds writes as well as reads, and only one RPC may run. */
    public byte[] readControl(byte[] command) throws IOException {
        if(command!=null&&command.length!=64)throw new IOException("Camera command must contain64 bytes");
        final InputStream stream;final OutputStream sink;final SSLSocket socket;final ScheduledExecutorService timer;
        synchronized(gate){
            if(closed||!controlMode||input==null||output==null)throw new IOException("Camera control is not connected");
            if(rpcPending||controlStarted!=(command!=null))throw new IOException("Camera RPC order or concurrency is invalid");
            if(rpcDeadline==null)rpcDeadline=Executors.newSingleThreadScheduledExecutor(task->{Thread t=new Thread(task,"CameraRpcDeadline");t.setDaemon(true);return t;});
            rpcPending=true;stream=input;sink=output;socket=tls;timer=rpcDeadline;
        }
        final AtomicBoolean expired=new AtomicBoolean();ScheduledFuture<?> expiry=null;
        final long until=System.nanoTime()+TimeUnit.MILLISECONDS.toNanos(250);
        try{
            expiry=timer.schedule(()->{expired.set(true);close();},250,TimeUnit.MILLISECONDS);
            if(command!=null){byte[] copy=command.clone();sink.write(copy);sink.flush();}
            byte[] response=readExactly(stream,96,until,socket);
            synchronized(gate){if(closed||expired.get())throw new SocketTimeoutException("Camera RPC deadline or connection ended");controlStarted=true;}
            return response;
        }catch(IOException error){close();if(expired.get())throw new SocketTimeoutException("Camera RPC deadline exceeded");throw error;}
        catch(RuntimeException error){close();throw new IOException("Camera RPC could not start",error);}
        finally{if(expiry!=null)expiry.cancel(false);synchronized(gate){rpcPending=false;}}
    }

    public byte[] readFrame() throws IOException {
        final InputStream stream;final SSLSocket socket;
        synchronized(gate){if(closed || input==null||controlMode)throw new IOException("Remote scene connection is not open");stream=input;socket=tls;}
        return readFrame(stream,socket);
    }
    public static byte[] readFrame(InputStream input) throws IOException {return readFrame(input,null);}
    private static byte[] readFrame(InputStream input,Socket socket) throws IOException {
        long deadline=System.nanoTime()+FRAME_MS*1000000L;
        byte[] header=readExactly(input,8,deadline,socket);
        long raw=unsigned32(header,0),compressed=unsigned32(header,4);
        if(raw<128 || raw>MAX_BYTES || compressed<1 || compressed>MAX_BYTES)throw new IOException("Remote scene transport length outside bounds");
        byte[] encoded=readExactly(input,(int)compressed,deadline,socket);
        return inflate(encoded,(int)raw);
    }
    private static long unsigned32(byte[] bytes,int at){
        long value=0;for(int i=0;i<4;i++)value=(value<<8)|(bytes[at+i]&255);return value;
    }
    private static byte[] readExactly(InputStream in,int size,long deadline,Socket socket) throws IOException {
        byte[] bytes=new byte[size];int used=0;
        while(used<size){
            long remaining=deadline-System.nanoTime();if(remaining<=0)throw new SocketTimeoutException("Remote scene frame deadline exceeded");
            if(socket!=null)socket.setSoTimeout((int)Math.min(READ_IDLE_MS,Math.max(1,(remaining+999999)/1000000)));
            int count=in.read(bytes,used,size-used);
            if(count<0)throw new EOFException("Remote scene frame was truncated or stream ended");
            if(count==0)throw new IOException("Remote scene stream made no progress");
            used+=count;
        }
        if(deadline-System.nanoTime()<=0)throw new SocketTimeoutException("Remote scene frame deadline exceeded");
        return bytes;
    }
    private static byte[] inflate(byte[] encoded,int expected) throws IOException {
        Inflater inflater=new Inflater();
        try {
            inflater.setInput(encoded);byte[] output=new byte[expected];int used=0;
            while(used<expected && !inflater.finished()){
                int count=inflater.inflate(output,used,expected-used);
                if(count==0){if(inflater.finished())break;throw new IOException("Incomplete or dictionary-compressed remote scene frame");}
                used+=count;
            }
            if(used!=expected)throw new IOException("Remote scene inflated length differs from header");
            if(!inflater.finished() && inflater.inflate(new byte[1])!=0)throw new IOException("Remote scene decompression exceeded declared length");
            if(!inflater.finished() || inflater.getRemaining()!=0)throw new IOException("Remote scene compressed stream has trailing or incomplete data");
            return output;
        }catch(DataFormatException error){throw new IOException("Invalid compressed remote scene frame",error);
        }finally{inflater.end();}
    }
    public static byte[] readRaw(InputStream input) throws IOException {
        byte[] bytes=readBounded(input,MAX_BYTES);if(bytes.length<128)throw new IOException("Remote scene asset is truncated");return bytes;
    }
    static byte[] readBounded(InputStream input,int limit) throws IOException {
        byte[] bytes=new byte[limit+1];int used=0;
        while(used<bytes.length){int count=input.read(bytes,used,bytes.length-used);if(count<0)break;
            if(count==0)throw new IOException("Remote scene input made no progress");used+=count;}
        if(used>limit)throw new IOException("Remote scene input exceeds its byte limit");return Arrays.copyOf(bytes,used);
    }
    @Override public void close(){
        Socket transport;SSLSocket secure;ScheduledExecutorService timer,rpcTimer;
        synchronized(gate){closed=true;transport=tcp;secure=tls;timer=deadline;rpcTimer=rpcDeadline;tcp=null;tls=null;input=null;output=null;deadline=null;rpcDeadline=null;}
        if(timer!=null)timer.shutdownNow();
        if(rpcTimer!=null)rpcTimer.shutdownNow();
        // Close the underlying transport first: TLS close_notify must not delay cancellation.
        if(transport!=null)try{transport.close();}catch(IOException ignored){}
        if(secure!=null)try{secure.close();}catch(IOException ignored){}
    }

    private static int hexDigit(char c){if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;}
    private static byte[] hex32(String text) throws IOException {
        if(text.length()!=64)throw new IOException("Remote scene pin/token must contain 64 hexadecimal characters");
        byte[] out=new byte[32];for(int i=0;i<32;i++){int a=hexDigit(text.charAt(i*2)),b=hexDigit(text.charAt(i*2+1));
            if(a<0||b<0)throw new IOException("Invalid hexadecimal remote scene credential");out[i]=(byte)((a<<4)|b);}return out;
    }
    private static byte[] ipv4(String text) throws IOException {
        String[] groups=text.split("\\.",-1);if(groups.length!=4)throw new IOException("Remote scene host must be a numeric IP address");
        byte[] out=new byte[4];
        for(int i=0;i<4;i++){String group=groups[i];if(group.isEmpty()||group.length()>3||(group.length()>1&&group.charAt(0)=='0'))throw new IOException("Invalid numeric IPv4 address");
            int n=0;for(char c:group.toCharArray()){if(c<'0'||c>'9')throw new IOException("Remote scene host must be numeric");n=n*10+c-'0';}
            if(n>255)throw new IOException("Invalid numeric IPv4 address");out[i]=(byte)n;}return out;
    }
    private static List<Integer> ipv6Groups(String text) throws IOException {
        List<Integer> result=new ArrayList<>();if(text.isEmpty())return result;
        String[] groups=text.split(":",-1);
        for(int i=0;i<groups.length;i++){
            String group=groups[i];if(group.indexOf('.')>=0){if(i!=groups.length-1)throw new IOException("Invalid numeric IPv6 tail");
                byte[] v4=ipv4(group);result.add(((v4[0]&255)<<8)|(v4[1]&255));result.add(((v4[2]&255)<<8)|(v4[3]&255));continue;}
            if(group.isEmpty()||group.length()>4)throw new IOException("Invalid numeric IPv6 address");int n=0;
            for(char c:group.toCharArray()){int digit=hexDigit(c);if(digit<0)throw new IOException("Remote scene host must be numeric without a zone");n=(n<<4)|digit;}result.add(n);
        }
        return result;
    }
    private static InetAddress literalAddress(String host) throws IOException {
        if(host.isEmpty()||host.length()>64)throw new IOException("Invalid numeric remote scene host");
        if(host.indexOf(':')<0)return InetAddress.getByAddress(ipv4(host));
        String[] halves=host.split("::",-1);if(halves.length>2 || (halves.length==2&&halves[0].indexOf('.')>=0))throw new IOException("Invalid numeric IPv6 address");
        List<Integer> left=ipv6Groups(halves[0]),right=halves.length==2?ipv6Groups(halves[1]):new ArrayList<>();
        int count=left.size()+right.size();if((halves.length==1&&count!=8)||(halves.length==2&&count>=8))throw new IOException("Invalid numeric IPv6 address");
        byte[] bytes=new byte[16];int at=0;for(int group:left){bytes[at++]=(byte)(group>>8);bytes[at++]=(byte)group;}
        at=16-right.size()*2;for(int group:right){bytes[at++]=(byte)(group>>8);bytes[at++]=(byte)group;}
        return InetAddress.getByAddress(bytes); // Never use a DNS-capable name lookup.
    }

    /** The provisioning schema is a flat JSON object of strings and integer literals. */
    private static final class FlatJson {
        private final String text;private int at;
        FlatJson(String text){this.text=text;}
        private IOException invalid(){return new IOException("Malformed remote scene configuration JSON");}
        private void space(){while(at<text.length()&&" \t\r\n".indexOf(text.charAt(at))>=0)at++;}
        private char next() throws IOException {if(at==text.length())throw invalid();return text.charAt(at++);}
        private String string() throws IOException {
            if(next()!='"')throw invalid();StringBuilder out=new StringBuilder();
            while(at<text.length()){
                char c=next();if(c=='"')return out.toString();if(c<32)throw invalid();
                if(c=='\\'){
                    c=next();switch(c){case '"':case '\\':case '/':break;case 'b':c='\b';break;case 'f':c='\f';break;
                        case 'n':c='\n';break;case 'r':c='\r';break;case 't':c='\t';break;
                        case 'u':int value=0;for(int i=0;i<4;i++){int digit=hexDigit(next());if(digit<0)throw invalid();value=(value<<4)|digit;}c=(char)value;break;
                        default:throw invalid();}
                }
                out.append(c);
            }
            throw invalid();
        }
        private Long integer() throws IOException {
            int start=at;if(at<text.length()&&text.charAt(at)=='-')at++;
            if(at>=text.length()||text.charAt(at)<'0'||text.charAt(at)>'9')throw invalid();
            if(text.charAt(at)=='0')at++;else while(at<text.length()&&text.charAt(at)>='0'&&text.charAt(at)<='9')at++;
            try{return Long.valueOf(text.substring(start,at));}catch(NumberFormatException error){throw invalid();}
        }
        Map<String,Object> parse() throws IOException {
            Map<String,Object> values=new HashMap<>();space();if(next()!='{')throw invalid();space();
            if(at<text.length()&&text.charAt(at)=='}'){at++;space();if(at!=text.length())throw invalid();return values;}
            while(true){
                String key=string();space();if(next()!=':')throw invalid();space();
                if(at>=text.length())throw invalid();Object value=text.charAt(at)=='"'?string():integer();
                if(values.put(key,value)!=null)throw new IOException("Duplicate remote scene configuration field");
                space();char delimiter=next();if(delimiter=='}')break;if(delimiter!=',')throw invalid();space();
            }
            space();if(at!=text.length())throw invalid();return values;
        }
    }
}
