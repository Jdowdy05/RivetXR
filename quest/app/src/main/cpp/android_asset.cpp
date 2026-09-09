#include "android_asset.h"

#include <android/asset_manager.h>

#include <array>
#include <filesystem>
#include <fstream>

namespace quest_newton {

bool ExtractAssetAtomically(AAssetManager* manager, const char* asset_name,
                            const char* internal_directory, const char* output_name,
                            std::string* error) {
    if (manager == nullptr || asset_name == nullptr || internal_directory == nullptr ||
        output_name == nullptr || error == nullptr || std::string(asset_name).find("..") != std::string::npos ||
        std::string(output_name).find_first_of("/\\") != std::string::npos) {
        if (error != nullptr) *error = "invalid asset extraction arguments";
        return false;
    }
    AAsset* asset = AAssetManager_open(manager, asset_name, AASSET_MODE_STREAMING);
    if (asset == nullptr) {
        *error = "asset is missing: " + std::string(asset_name);
        return false;
    }
    const std::filesystem::path directory(internal_directory);
    const std::filesystem::path target = directory / output_name;
    const std::filesystem::path temporary = directory / (std::string(output_name) + ".tmp");
    std::error_code fs_error;
    const auto remove_temporary = [&]() { std::filesystem::remove(temporary, fs_error); };
    std::filesystem::create_directories(directory, fs_error);
    if (fs_error) {
        AAsset_close(asset);
        *error = "cannot create internal asset directory: " + fs_error.message();
        return false;
    }
    const off64_t expected_length = AAsset_getLength64(asset);
    if (expected_length < 0) {
        AAsset_close(asset);
        *error = "asset length is unavailable";
        return false;
    }
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
        AAsset_close(asset);
        *error = "cannot open temporary internal asset file";
        return false;
    }
    std::array<char, 16 * 1024> buffer{};
    int read = 0;
    off64_t written = 0;
    while ((read = AAsset_read(asset, buffer.data(), buffer.size())) > 0) {
        output.write(buffer.data(), read);
        written += read;
        if (!output) {
            output.close();
            AAsset_close(asset);
            remove_temporary();
            *error = "cannot write temporary internal asset file";
            return false;
        }
    }
    output.flush();
    const bool flushed = static_cast<bool>(output);
    output.close();
    const bool closed = static_cast<bool>(output);
    AAsset_close(asset);
    if (read < 0 || written != expected_length || !flushed || !closed) {
        remove_temporary();
        *error = "asset read or flush was incomplete";
        return false;
    }
    std::filesystem::rename(temporary, target, fs_error);
    if (fs_error) {
        std::filesystem::remove(temporary, fs_error);
        *error = "cannot atomically publish internal asset: " + fs_error.message();
        return false;
    }
    return true;
}

}  // namespace quest_newton
