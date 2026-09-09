#pragma once

#include <android/asset_manager.h>

#include <string>

namespace quest_newton {

bool ExtractAssetAtomically(AAssetManager* manager, const char* asset_name,
                            const char* internal_directory, const char* output_name,
                            std::string* error);

}  // namespace quest_newton
