package com.questnewton;

import java.nio.file.Files;
import java.nio.file.Path;
import java.security.KeyManagementException;
import java.security.Provider;
import java.security.SecureRandom;
import java.security.Security;
import javax.net.ssl.KeyManager;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLContextSpi;
import javax.net.ssl.SSLEngine;
import javax.net.ssl.SSLServerSocketFactory;
import javax.net.ssl.SSLSessionContext;
import javax.net.ssl.SSLSocketFactory;
import javax.net.ssl.TrustManager;

/** Host-only cold-provider fixture, using the real Python control service. */
public final class SlowTlsInitialization {
    private static SSLContext delegate;
    public static final class SlowContext extends SSLContextSpi {
        @Override protected void engineInit(KeyManager[] keys,TrustManager[] trust,SecureRandom random) throws KeyManagementException {
            try{Thread.sleep(2300);}catch(InterruptedException error){Thread.currentThread().interrupt();throw new KeyManagementException(error);}
            delegate.init(keys,trust,random);
        }
        @Override protected SSLSocketFactory engineGetSocketFactory(){return delegate.getSocketFactory();}
        @Override protected SSLServerSocketFactory engineGetServerSocketFactory(){return delegate.getServerSocketFactory();}
        @Override protected SSLEngine engineCreateSSLEngine(){return delegate.createSSLEngine();}
        @Override protected SSLEngine engineCreateSSLEngine(String host,int port){return delegate.createSSLEngine(host,port);}
        @Override protected SSLSessionContext engineGetClientSessionContext(){return delegate.getClientSessionContext();}
        @Override protected SSLSessionContext engineGetServerSessionContext(){return delegate.getServerSessionContext();}
    }
    private static final class SlowProvider extends Provider {
        private static final long serialVersionUID=1L;
        SlowProvider(){super("SlowTlsFixture","1.0","Bounded TLS initialization fixture");put("SSLContext.TLS",SlowContext.class.getName());}
    }
    public static void main(String[] args) throws Exception {
        // Warm the delegate itself so only the controlled initialization pause
        // consumes the unchanged four-second connection-setup budget.
        delegate=SSLContext.getInstance("TLS");delegate.init(null,null,null);
        delegate.getSocketFactory().createSocket().close();
        Provider provider=new SlowProvider();Security.insertProviderAt(provider,1);
        try(RemoteSceneWire wire=new RemoteSceneWire()){
            wire.connect(RemoteSceneWire.Config.parse(Files.readString(Path.of(args[0]))),"QGC1");
            byte[] feedback=wire.readControl(null);
            if(feedback.length!=96||feedback[0]!='Q'||feedback[1]!='G'||feedback[2]!='C'||feedback[3]!='F')
                throw new AssertionError("Missing real control-service greeting after slow TLS initialization");
        }finally{Security.removeProvider(provider.getName());}
        System.out.println("slow local TLS initialization leaves peer authentication budget intact");
    }
}
