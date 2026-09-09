#include "quest_newton_app.h"

#include <android/log.h>
#include <android_native_app_glue.h>

namespace {
bool ReadLaunchOptions(ANativeActivity* activity, quest_newton::verification::LaunchOptions& options,
                       std::string& error) {
    JNIEnv* env = nullptr;
    bool attached_here = false;
    const auto env_result = activity->vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
    if (env_result == JNI_EDETACHED) {
        if (activity->vm->AttachCurrentThread(&env, nullptr) != JNI_OK) {
            error = "could not attach launch-option JNI thread";
            return false;
        }
        attached_here = true;
    } else if (env_result != JNI_OK || env == nullptr) {
        error = "launch-option JNI environment unavailable";
        return false;
    }
    bool pushed = false;
    const auto cleanup = [&] {
        if (env->ExceptionCheck()) env->ExceptionClear();
        if (pushed) env->PopLocalFrame(nullptr);
        if (attached_here) activity->vm->DetachCurrentThread();
    };
    if (env->PushLocalFrame(32) != JNI_OK) {
        error = "launch-option JNI local frame failed";
        cleanup();
        return false;
    }
    pushed = true;
    const auto fail = [&](const char* detail) { error = detail; cleanup(); return false; };
    jclass activity_class = env->GetObjectClass(activity->clazz);
    if (activity_class == nullptr) return fail("activity class unavailable");
    jmethodID get_intent = env->GetMethodID(activity_class, "getIntent", "()Landroid/content/Intent;");
    if (get_intent == nullptr) return fail("getIntent unavailable");
    jobject intent = env->CallObjectMethod(activity->clazz, get_intent);
    if (env->ExceptionCheck() || intent == nullptr) return fail("getIntent failed");
    jclass intent_class = env->GetObjectClass(intent);
    if (intent_class == nullptr) return fail("intent class unavailable");
    jmethodID get_string = env->GetMethodID(intent_class, "getStringExtra", "(Ljava/lang/String;)Ljava/lang/String;");
    if (get_string == nullptr) return fail("getStringExtra unavailable");
    jmethodID get_int = env->GetMethodID(intent_class, "getIntExtra", "(Ljava/lang/String;I)I");
    if (get_int == nullptr) return fail("getIntExtra unavailable");
    const auto read_string = [&](const char* key_name, std::string& value) {
        jstring key = env->NewStringUTF(key_name);
        if (key == nullptr) return false;
        auto text = static_cast<jstring>(env->CallObjectMethod(intent, get_string, key));
        if (env->ExceptionCheck()) return false;
        if (text == nullptr) { value.clear(); return true; }
        const char* data = env->GetStringUTFChars(text, nullptr);
        if (data == nullptr) return false;
        value = data;
        env->ReleaseStringUTFChars(text, data);
        return true;
    };
    std::string mode, run_id;
    if (!read_string("quest_newton_mode", mode) || !read_string("quest_newton_run_id", run_id)) {
        return fail("reading launch string extras failed");
    }
    jstring duration_key = env->NewStringUTF("quest_newton_soak_seconds");
    if (duration_key == nullptr) return fail("allocating duration key failed");
    const jint duration = env->CallIntMethod(intent, get_int, duration_key, 1800);
    if (env->ExceptionCheck()) return fail("reading soak duration failed");
    const auto read_int = [&](const char* name, int fallback) {
        jstring key = env->NewStringUTF(name);
        return key == nullptr ? -1 : env->CallIntMethod(intent, get_int, key, fallback);
    };
    const int hz = read_int("quest_newton_benchmark_hz", 200);
    const int seconds = read_int("quest_newton_benchmark_seconds", 30);
    const int warmup = read_int("quest_newton_benchmark_warmup_seconds", 8);
    std::string workload;
    if (env->ExceptionCheck() || !read_string("quest_newton_benchmark_workload", workload))
        return fail("reading benchmark extras failed");
    if (workload.empty()) workload = "motion";
    const bool result = quest_newton::verification::ParseLaunchOptions(mode, run_id, duration, options, error,
        hz, seconds, warmup, workload);
    cleanup();
    return result;
}
} // namespace

extern "C" void android_main(struct android_app* app) {
    if (app == nullptr || app->activity == nullptr) return;
    __android_log_print(ANDROID_LOG_INFO, "QuestNewton", "%s", "QUEST_NEWTON_STAGE native_entry");
    quest_newton::verification::LaunchOptions options;
    std::string error;
    if (!ReadLaunchOptions(app->activity, options, error)) {
        __android_log_print(ANDROID_LOG_ERROR, "QuestNewton", "QUEST_NEWTON_VERIFY_FAIL stage=launch_options detail=%s", error.c_str());
        ANativeActivity_finish(app->activity);
        return;
    }
    quest_newton::QuestNewtonApp application(app->activity->assetManager, app->activity->internalDataPath, options);
    application.Run(app);
}
