package com.questnewton;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

/** Loopback simulation fixture. Output excludes TLS credentials. */
public final class GimbalPythonInterop {
    static ByteBuffer view(byte[] b){return ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);}
    static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    static void stamp(ByteArrayOutputStream out,long n){out.writeBytes(ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN).putLong(n).array());}
    public static void main(String[] args) throws Exception {
        RemoteSceneWire.Config config=RemoteSceneWire.Config.parse(Files.readString(Path.of(args[0])));
        try(RemoteSceneWire wire=new RemoteSceneWire()){
            if(args[1].equals("reject")){
                try{wire.connect(config,"QGC1");wire.readControl(null);}catch(IOException expected){System.out.println("bad control credentials rejected");return;}
                throw new AssertionError("bad control credentials accepted");
            }
            wire.connect(config,"QGC1");byte[] feedback=wire.readControl(null);long received=System.nanoTime();
            check(Arrays.equals(Arrays.copyOf(feedback,4),new byte[]{'Q','G','C','F'}),"Python feedback magic differs");
            check(view(feedback).getLong(24)==0&&(view(feedback).getInt(56)&15)==5,"Python initial session is not simulated, tracked, and unarmed");
            ByteArrayOutputStream transcript=new ByteArrayOutputStream();transcript.writeBytes(new byte[]{'Q','G','I','T',1,0,0,0,7,0,0,0});
            stamp(transcript,received);transcript.writeBytes(feedback);long oldAimSample=0;
            for(int sequence=1;sequence<=7;++sequence){
                if(sequence>1)Thread.sleep(sequence==3?170:50);
                long sent=System.nanoTime();long sample=sequence==3?oldAimSample:sent;if(sequence==2)oldAimSample=sample;
                boolean aim=sequence==2||sequence==6;ByteBuffer previous=view(feedback);
                ByteBuffer command=ByteBuffer.allocate(64).order(ByteOrder.LITTLE_ENDIAN);
                command.put(new byte[]{'Q','G','C','M'}).putShort((short)1).putShort((short)64);
                command.putLong(previous.getLong(8)).putLong(previous.getLong(16)).putLong(sequence).putLong(sent);
                command.putInt(aim?1:0).putInt(aim?1:0).putFloat(aim?.25F:previous.getFloat(40)).putFloat(aim?.1F:previous.getFloat(44)).putInt(100).putInt(0);
                byte[] response=wire.readControl(command.array());received=System.nanoTime();ByteBuffer next=view(response);
                check(next.getLong(8)==previous.getLong(8)&&next.getLong(48)==77&&next.getLong(24)==sequence&&next.getLong(16)!=previous.getLong(16),"Python acknowledgement or identity differs");
                check(Long.compareUnsigned(next.getLong(32),previous.getLong(32))>=0,"Python server clock moved backward");
                check(((next.getInt(56)&2)!=0)==aim,"Python simulated arm/hold lease mismatch");
                if(sequence==3)check(Math.abs(next.getFloat(40))<=.15F,"simulated lease failed to stop between requests");
                stamp(transcript,sample);stamp(transcript,sent);transcript.writeBytes(command.array());stamp(transcript,received);transcript.writeBytes(response);feedback=response;
            }
            Files.write(Path.of(args[2]),transcript.toByteArray());
        }
        System.out.println("Python Java control hold aim expiry release and disconnect passed");
    }
}
