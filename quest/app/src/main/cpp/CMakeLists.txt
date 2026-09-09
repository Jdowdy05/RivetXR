cmake_minimum_required(VERSION 3.22.1)
project(quest_newton LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT ANDROID)
    message(FATAL_ERROR "Quest Newton APK must be built with the Android toolchain")
endif()
if(NOT ANDROID_ABI STREQUAL "arm64-v8a")
    message(FATAL_ERROR "Quest Newton only supports arm64-v8a")
endif()
foreach(required_var META_OPENXR_SDK_ROOT QUEST_NEWTON_REPO_ROOT QUEST_NEWTON_ARTIFACT_DIR)
    if(NOT DEFINED ${required_var} OR "${${required_var}}" STREQUAL "")
        message(FATAL_ERROR "${required_var} is required")
    endif()
endforeach()

set(META_3RDPARTY "${META_OPENXR_SDK_ROOT}/Samples/3rdParty")
set(META_FRAMEWORK "${META_OPENXR_SDK_ROOT}/Samples/SampleXrFramework")
if(NOT EXISTS "${META_3RDPARTY}/CMakeLists.txt" OR NOT EXISTS "${META_FRAMEWORK}/CMakeLists.txt")
    message(FATAL_ERROR "Meta v85 Samples/3rdParty and Samples/SampleXrFramework are required")
endif()

add_subdirectory("${META_3RDPARTY}" "meta_3rdparty")
add_subdirectory("${META_FRAMEWORK}" "meta_framework")
include("${QUEST_NEWTON_REPO_ROOT}/cmake/CompilerWarnings.cmake")
add_subdirectory("${QUEST_NEWTON_REPO_ROOT}/src/newton_runtime" "quest_newton_runtime")
add_subdirectory("${QUEST_NEWTON_REPO_ROOT}/src/kinematics" "quest_newton_kinematics")

add_library(quest_newton SHARED
    native_main.cpp android_asset.cpp quest_newton_app.cpp
    arm_renderer.cpp passthrough_layer.cpp input_mapper.cpp mesh_geometry.cpp
    gpu_timer.cpp verification_worker.cpp
)
target_include_directories(quest_newton PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${META_FRAMEWORK}/Src"
    "${QUEST_NEWTON_ARTIFACT_DIR}/include"
    "${QUEST_NEWTON_REPO_ROOT}/src/newton_runtime/include"
)
option(QUEST_NEWTON_FULL_RUNTIME "Embed the complete Python/Newton CPU robot runtime" OFF)
if(QUEST_NEWTON_FULL_RUNTIME)
    target_compile_definitions(quest_newton PRIVATE QUEST_NEWTON_FULL_RUNTIME=1)
    target_sources(quest_newton PRIVATE
        full_runtime_app.cpp full_benchmark.cpp full_control.cpp interaction_trace.cpp python_session.cpp sim_menu.cpp menu_pointer_renderer.cpp
        sim_settings.cpp base_registration.cpp registered_input.cpp
        scene_snapshot.cpp environment_renderer.cpp room_environment.cpp room_scene.cpp)
    target_sources(quest_newton PRIVATE object_interaction.cpp scene_details.cpp)
    target_sources(quest_newton PRIVATE remote_inspection.cpp remote_scene_app.cpp)
    target_sources(quest_newton PRIVATE remote_scene.cpp remote_surface.cpp remote_rgb_surface.cpp remote_scene_renderer.cpp gimbal_control.cpp)
    target_sources(quest_newton PRIVATE remote_scene_receiver.cpp gimbal_client.cpp)
endif()
target_compile_definitions(quest_newton PRIVATE ANDROID_STL=c++_shared)
target_compile_options(quest_newton PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
target_link_options(quest_newton PRIVATE "-u" "ANativeActivity_onCreate")
target_link_libraries(quest_newton PRIVATE
    samplexrframework quest_newton_runtime quest_newton_kinematics android EGL GLESv3 log dl
)
set_target_properties(quest_newton PROPERTIES OUTPUT_NAME "quest_newton")
