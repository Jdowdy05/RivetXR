package com.questnewton;

import android.content.Context;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;

/** Constructed before native Start; all config/asset/network reads run on its receive worker. */
public final class RemoteSceneConnection implements AutoCloseable {
    private final Context context;
    private final boolean demo;
    private final int demoMode;
    private final RemoteSceneWire wire=new RemoteSceneWire();
    private final Object gate=new Object();
    private boolean closed,delivered;
    private InputStream asset;

    public RemoteSceneConnection(Context context,boolean demo) {
        this(context,demo?1:0);
    }
    public RemoteSceneConnection(Context context,int demoMode) {
        if(demoMode<0 || demoMode>5)throw new IllegalArgumentException("Unknown remote demo representation");
        this.context=context.getApplicationContext();this.demoMode=demoMode;this.demo=demoMode!=0;
    }
    public void connect() throws IOException {
        synchronized(gate){if(closed)throw new IOException("Remote scene connection is closed");}
        if(demo){
            InputStream opened;
            String name=demoMode==4?"demo-rgb-prepared.rscn":demoMode==5?"demo-rgb-unprepared.rscn":
                demoMode==2?"demo-prepared.rscn":demoMode==3?"demo-unprepared.rscn":"demo.rscn";
            try {opened=context.getAssets().open("remote_scene/"+name);}
            catch(IOException error){throw new IOException("Remote scene demo asset is unavailable");}
            synchronized(gate){if(closed){opened.close();throw new IOException("Remote scene demo was cancelled");}asset=opened;}
            return;
        }
        RemoteSceneWire.Config config;
        try(InputStream input=new FileInputStream(new File(context.getFilesDir(),"remote-scene-client.json"))){
            byte[] bytes=RemoteSceneWire.readBounded(input,RemoteSceneWire.MAX_CONFIG_BYTES);
            String text=StandardCharsets.UTF_8.newDecoder().onMalformedInput(CodingErrorAction.REPORT)
                .onUnmappableCharacter(CodingErrorAction.REPORT).decode(ByteBuffer.wrap(bytes)).toString();
            config=RemoteSceneWire.Config.parse(text);
        }catch(IOException error){throw new IOException("Remote scene configuration is unavailable or invalid");}
        try {wire.connect(config);}catch(IOException error){throw safeError(error,"Remote scene TLS connection failed");}
    }
    public byte[] readFrame() throws IOException {
        if(!demo){try{return wire.readFrame();}catch(IOException error){throw safeError(error,"Remote scene receive failed");}}
        InputStream stream;
        synchronized(gate){
            if(closed)throw new IOException("Remote scene demo was cancelled");
            if(delivered)return null;
            if(asset==null)throw new IOException("Remote scene demo is not open");
            delivered=true;stream=asset;
        }
        byte[] result=RemoteSceneWire.readRaw(stream);
        synchronized(gate){if(closed)throw new IOException("Remote scene demo was cancelled");}
        return result;
    }
    private static IOException safeError(IOException error,String fallback){
        // Network/provider messages may contain a configured endpoint. Only
        // fixed reasons and exception type names cross into UI/log diagnostics.
        Throwable cause=error;
        for(int i=0;cause!=null&&i<8;i++,cause=cause.getCause()){
            if("Remote scene certificate pin mismatch".equals(cause.getMessage()))return new IOException("Remote scene certificate pin mismatch");
            if(cause instanceof java.security.cert.CertificateExpiredException)return new IOException("Remote scene certificate expired");
            if(cause instanceof java.net.SocketTimeoutException)return new IOException("Remote scene connection or frame deadline exceeded");
        }
        return new IOException(fallback+" ("+error.getClass().getSimpleName()+")");
    }
    @Override public void close(){
        InputStream stream;
        synchronized(gate){closed=true;stream=asset;asset=null;}
        wire.close();if(stream!=null)try{stream.close();}catch(IOException ignored){}
    }
}
