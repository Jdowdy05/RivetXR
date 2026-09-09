package android.content;
import android.content.res.AssetManager;
import java.io.File;
/** Host-only adapter fixture; never part of APK sources. */
public abstract class Context {
    public Context getApplicationContext(){return this;}
    public abstract File getFilesDir();
    public abstract AssetManager getAssets();
}
