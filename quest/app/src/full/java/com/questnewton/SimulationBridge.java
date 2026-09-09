package com.questnewton;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.util.Log;
import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import org.json.JSONObject;

/** Called exclusively by the native simulation worker, never the XR render loop. */
public final class SimulationBridge {
    private PyObject session;
    private final String evidenceRunId = UUID.randomUUID().toString();

    public SimulationBridge(Context context, String settings) throws IOException {
        Context application = context.getApplicationContext();
        if (!Python.isStarted()) Python.start(new AndroidPlatform(application));
        Python python = Python.getInstance();
        File assets = new File(application.getFilesDir(), "full-runtime-assets");
        copyAssets(application.getAssets(), "full_runtime", assets);
        PyObject runtime = python.getModule("quest_sim.runtime");
        runtime.callAttr("bootstrap", application.getApplicationInfo().nativeLibraryDir,
                new File(application.getCacheDir(), "newton-warp").getAbsolutePath());
        // Prove on-device compilation before importing the complete robot scene.
        String jit = python.getModule("quest_sim.smoke").callAttr("run").toString();
        Log.i("QuestNewton", "QUEST_FULL_JIT " + jit);
        writeEvidence(application, "full-jit.json", jit);
        if (context instanceof Activity && ((Activity) context).getIntent() != null
                && ((Activity) context).getIntent().getBooleanExtra("quest_newton_cpu_verify", false)) {
            String proof = python.getModule("quest_sim.cpu_validation")
                    .callAttr("run", assets.getAbsolutePath()).toString();
            // Verification evidence is required, unlike optional startup diagnostics.
            writeRequiredEvidence(application, "full-cpu-equivalence.json", proof);
            try {
                if (!new JSONObject(proof).getBoolean("passed")) {
                    throw new IOException("CPU equivalence verification failed; inspect full-cpu-equivalence.json");
                }
            } catch (org.json.JSONException error) {
                throw new IOException("Invalid CPU equivalence proof", error);
            }
            Log.i("QuestNewton", "QUEST_FULL_CPU_EQUIVALENCE passed");
        }
        if (context instanceof Activity && ((Activity) context).getIntent() != null
                && ((Activity) context).getIntent().getBooleanExtra("quest_newton_scene_verify", false)) {
            String proof = python.getModule("quest_sim.scene_validation")
                    .callAttr("run", assets.getAbsolutePath()).toString();
            writeRequiredEvidence(application, "full-scene-verification.json", proof);
            try {
                if (!new JSONObject(proof).getBoolean("passed")) {
                    throw new IOException("Scene verification failed; inspect full-scene-verification.json");
                }
            } catch (org.json.JSONException error) {
                throw new IOException("Invalid scene verification proof", error);
            }
            Log.i("QuestNewton", "QUEST_FULL_SCENE_VERIFICATION passed");
        }
        if (context instanceof Activity && ((Activity) context).getIntent() != null
                && ((Activity) context).getIntent().getBooleanExtra("quest_newton_gripper_verify", false)) {
            // Explicit diagnostic model selection never modifies live/saved settings.
            String contactProfile = ((Activity) context).getIntent().getStringExtra("quest_newton_gripper_profile");
            String proof = python.getModule("quest_sim.gripper_validation")
                    .callAttr("run", assets.getAbsolutePath(), contactProfile).toString();
            writeRequiredEvidence(application, "full-gripper-verification.json", proof);
            try {
                if (!new JSONObject(proof).getBoolean("passed")) {
                    throw new IOException("Gripper verification failed; inspect full-gripper-verification.json");
                }
            } catch (org.json.JSONException error) {
                throw new IOException("Invalid gripper verification proof", error);
            }
            Log.i("QuestNewton", "QUEST_FULL_GRIPPER_VERIFICATION passed");
        }
        session = runtime.callAttr("create", assets.getAbsolutePath(), settings);
        String metadata = session.callAttr("metadata").toString();
        Log.i("QuestNewton", "QUEST_FULL_RUNTIME " + metadata);
        writeEvidence(application, "full-runtime-metadata.json", metadata);
    }

    public byte[] snapshot() { return session.callAttr("snapshot").toJava(byte[].class); }
    public byte[] details(boolean contacts) { return session.callAttr("details_bytes", contacts).toJava(byte[].class); }
    public byte[] step(double dt, float[] targets, float gripper, boolean gripperInputAllowed) {
        return session.callAttr("step", dt, targets, gripper, gripperInputAllowed).toJava(byte[].class);
    }
    public byte[] command(String name) { return session.callAttr("command", name).toJava(byte[].class); }
    public void configure(String settings) { session.callAttr("configure", settings); }
    public byte[] setBasePose(float[] pose) { return session.callAttr("set_base_pose", pose).toJava(byte[].class); }
    public byte[] setEnvironment(String room) { return session.callAttr("set_environment", room).toJava(byte[].class); }
    public String metadata() { return session.callAttr("metadata").toString(); }
    public String gripperStatus() { return session.callAttr("gripper_status").toString(); }
    public void close() { if (session != null) { session.close(); session = null; } }

    public static boolean hasScenePermission(Context context) {
        return context.checkSelfPermission("com.oculus.permission.USE_SCENE") == PackageManager.PERMISSION_GRANTED;
    }

    public static void requestScenePermission(Activity activity) {
        // The native XR owner requests once per user enable/refresh attempt.
        activity.runOnUiThread(() -> {
            if (!activity.isFinishing() && !hasScenePermission(activity)) {
                activity.requestPermissions(new String[]{"com.oculus.permission.USE_SCENE"}, 4201);
            }
        });
    }

    private void writeEvidence(Context context, String name, String payload) {
        try {
            writeRequiredEvidence(context, name, payload);
        } catch (IOException error) {
            // Diagnostics must not prevent a valid physics session from starting.
            Log.w("QuestNewton", "QUEST_FULL_EVIDENCE_WRITE_FAIL " + name, error);
        }
    }

    private void writeRequiredEvidence(Context context, String name, String payload) throws IOException {
        try {
            JSONObject evidence = new JSONObject();
            evidence.put("pid", android.os.Process.myPid());
            evidence.put("run_id", evidenceRunId);
            evidence.put("monotonic_ms", android.os.SystemClock.elapsedRealtime());
            evidence.put("result", new JSONObject(payload));
            File target = new File(context.getFilesDir(), name);
            File temporary = new File(target.getPath() + ".pending");
            Files.write(temporary.toPath(), evidence.toString().getBytes(StandardCharsets.UTF_8));
            Files.move(temporary.toPath(), target.toPath(), StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE);
        } catch (org.json.JSONException error) {
            throw new IOException("Invalid evidence JSON: " + name, error);
        }
    }

    private static void copyAssets(AssetManager manager, String source, File target) throws IOException {
        String[] children = manager.list(source);
        if (children == null) throw new IOException("Missing scene asset: " + source);
        if (children.length > 0) {
            if (!target.isDirectory() && !target.mkdirs()) throw new IOException("Cannot create scene directory");
            for (String name : children) {
                if (name.contains("/") || name.equals(".") || name.equals("..")) throw new IOException("Invalid asset name");
                copyAssets(manager, source + "/" + name, new File(target, name));
            }
            return;
        }
        File temporary = new File(target.getPath() + ".pending");
        try (InputStream in = manager.open(source); FileOutputStream out = new FileOutputStream(temporary)) {
            byte[] buffer = new byte[65536];
            int count;
            while ((count = in.read(buffer)) != -1) out.write(buffer, 0, count);
            out.getFD().sync();
        }
        Files.move(temporary.toPath(), target.toPath(), StandardCopyOption.REPLACE_EXISTING,
                StandardCopyOption.ATOMIC_MOVE);
    }
}
