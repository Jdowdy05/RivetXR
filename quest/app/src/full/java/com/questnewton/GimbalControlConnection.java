package com.questnewton;
import android.content.Context;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;

/** Private-config simulation control. Constructor does no file/network work. */
public final class GimbalControlConnection implements AutoCloseable {
    private final Context context;
    private final RemoteSceneWire wire=new RemoteSceneWire();
    private final Object gate=new Object();
    private boolean closed;
    public GimbalControlConnection(Context context){this.context=context.getApplicationContext();}
    public byte[] open() throws IOException {
        synchronized(gate){if(closed)throw new IOException("Camera control connection was cancelled");}
        RemoteSceneWire.Config config;
        try(InputStream in=new FileInputStream(new File(context.getFilesDir(),"remote-gimbal-client.json"))){
            byte[] bytes=RemoteSceneWire.readBounded(in,RemoteSceneWire.MAX_CONFIG_BYTES);
            String text=StandardCharsets.UTF_8.newDecoder().onMalformedInput(CodingErrorAction.REPORT)
                .onUnmappableCharacter(CodingErrorAction.REPORT).decode(ByteBuffer.wrap(bytes)).toString();
            config=RemoteSceneWire.Config.parse(text);
        }catch(IOException error){throw new IOException("Camera control configuration is unavailable or invalid");}
        try{wire.connect(config,"QGC1");return wire.readControl(null);}
        catch(IOException error){wire.close();throw safe(error);}
    }
    public byte[] exchange(byte[] command) throws IOException {
        try{return wire.readControl(command);}catch(IOException error){throw safe(error);}
    }
    private static IOException safe(IOException error){
        Throwable cause=error;
        for(int i=0;cause!=null&&i<8;i++,cause=cause.getCause()){
            if("Remote scene certificate pin mismatch".equals(cause.getMessage()))return new IOException("Camera control certificate pin mismatch");
            if(cause instanceof java.net.SocketTimeoutException)return new IOException("Camera control deadline exceeded");
        }
        return new IOException("Camera control transport failed ("+error.getClass().getSimpleName()+")");
    }
    @Override public void close(){synchronized(gate){closed=true;}wire.close();}
}
