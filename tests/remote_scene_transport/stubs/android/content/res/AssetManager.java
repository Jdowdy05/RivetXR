package android.content.res;
import java.io.IOException;
import java.io.InputStream;
/** Host-only adapter fixture; never part of APK sources. */
public abstract class AssetManager {
    public abstract InputStream open(String name) throws IOException;
}
