package com.questnewton;

import android.content.Context;
import android.content.res.AssetManager;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public final class RemoteSceneConnectionTest {
    static void check(boolean value,String reason){if(!value)throw new AssertionError(reason);}
    interface Action {void run() throws Exception;}
    static void rejects(Action action) throws Exception {
        try{action.run();}catch(IOException expected){return;}throw new AssertionError("invalid adapter operation accepted");
    }
    static final class Fixture extends Context {
        final File files;InputStream stream;int opened,closed,fileAccess;String openingThread,expectedAsset="demo.rscn";
        Fixture(Path root,byte[] bytes){files=root.toFile();stream=new ByteArrayInputStream(bytes){@Override public void close(){closed++;}};}
        @Override public File getFilesDir(){fileAccess++;return files;}
        @Override public AssetManager getAssets(){return new AssetManager(){@Override public InputStream open(String name){
            check(name.equals("remote_scene/"+expectedAsset),"unexpected demo asset path");opened++;openingThread=Thread.currentThread().getName();return stream;
        }};}
    }
    public static void main(String[] args) throws Exception {
        Path root=Path.of(args[0]);byte[] bytes=new byte[128];bytes[0]='R';bytes[1]='S';bytes[2]='C';bytes[3]='N';
        Fixture fixture=new Fixture(root,bytes);RemoteSceneConnection demo=new RemoteSceneConnection(fixture,true);
        check(fixture.opened==0&&fixture.fileAccess==0,"constructor performed asset/config I/O before native ownership");
        AtomicReference<Throwable> error=new AtomicReference<>();
        Thread worker=new Thread(()->{try{demo.connect();check(Arrays.equals(demo.readFrame(),bytes),"demo packet bytes");
            check(demo.readFrame()==null,"demo did not end after one frame");}catch(Throwable failure){error.set(failure);}},"RemoteFixtureWorker");
        worker.start();worker.join(1500);check(!worker.isAlive()&&error.get()==null,"demo worker failed");demo.close();
        check(fixture.opened==1&&fixture.closed==1&&fixture.fileAccess==0&&"RemoteFixtureWorker".equals(fixture.openingThread),"demo I/O ownership");
        String[] assets={"demo.rscn","demo-prepared.rscn","demo-unprepared.rscn","demo-rgb-prepared.rscn","demo-rgb-unprepared.rscn"};
        for(int mode=1;mode<=5;mode++){
            Fixture assetFixture=new Fixture(root,bytes);assetFixture.expectedAsset=assets[mode-1];
            RemoteSceneConnection selected=new RemoteSceneConnection(assetFixture,mode);selected.connect();
            check(Arrays.equals(selected.readFrame(),bytes),"selected demo bytes");selected.close();
        }
        Fixture cancelled=new Fixture(root,bytes);RemoteSceneConnection beforeStart=new RemoteSceneConnection(cancelled,true);
        beforeStart.close();rejects(beforeStart::connect);check(cancelled.opened==0,"cancel-before-connect opened asset");

        Fixture blocked=new Fixture(root,bytes);CountDownLatch reading=new CountDownLatch(1),closed=new CountDownLatch(1);
        blocked.stream=new InputStream(){@Override public int read() throws IOException {throw new IOException("use bulk read");}
            @Override public int read(byte[] out,int offset,int length) throws IOException {
                reading.countDown();try{closed.await(3,TimeUnit.SECONDS);}catch(InterruptedException e){Thread.currentThread().interrupt();}
                throw new IOException("closed fixture stream");
            }
            @Override public void close(){closed.countDown();}
        };
        RemoteSceneConnection blockedDemo=new RemoteSceneConnection(blocked,true);blockedDemo.connect();error.set(null);
        Thread reader=new Thread(()->{try{blockedDemo.readFrame();}catch(Throwable failure){error.set(failure);}});reader.start();
        check(reading.await(1,TimeUnit.SECONDS),"demo read not reached");blockedDemo.close();reader.join(1500);
        check(!reader.isAlive()&&error.get() instanceof IOException,"close did not interrupt demo asset read");

        Fixture network=new Fixture(root,bytes);RemoteSceneConnection missing=new RemoteSceneConnection(network,false);
        check(network.fileAccess==0,"network constructor read config");rejects(missing::connect);missing.close();
        check(network.fileAccess==1&&network.opened==0,"network mode did not use private config only");
        Path config=root.resolve("remote-scene-client.json");Files.writeString(config," ".repeat(4097));
        RemoteSceneConnection oversized=new RemoteSceneConnection(network,false);rejects(oversized::connect);oversized.close();
        Files.writeString(config,"{\"version\":1,\"version\":1}");
        RemoteSceneConnection duplicate=new RemoteSceneConnection(network,false);rejects(duplicate::connect);duplicate.close();Files.delete(config);
        System.out.println("adapter constructor/worker/demo/config/close lifecycle passed");
    }
}
