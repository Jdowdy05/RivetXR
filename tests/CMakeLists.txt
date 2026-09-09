add_test(
  NAME repository_contract
  COMMAND
    "${CMAKE_COMMAND}"
    "-DNEWTON_VERSION=${NEWTON_VERSION}"
    "-DWARP_VERSION=${WARP_VERSION}"
    "-DNEWTON_COMMIT=${NEWTON_COMMIT}"
    "-DWARP_COMMIT=${WARP_COMMIT}"
    -P
    "${CMAKE_CURRENT_SOURCE_DIR}/repository_contract.cmake"
)
if(BUILD_TESTING)
  add_executable(remote_scene_test remote_scene_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_scene.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_surface.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_rgb_surface.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_scene_renderer.cpp")
  target_include_directories(remote_scene_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(remote_scene_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(remote_scene_test)
  if(MSVC)
    target_compile_options(remote_scene_test PRIVATE /EHsc)
  endif()
  add_test(NAME remote_scene COMMAND remote_scene_test)
  add_executable(remote_surface_test remote_surface_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_scene.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_surface.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_rgb_surface.cpp")
  target_include_directories(remote_surface_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(remote_surface_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(remote_surface_test)
  if(MSVC)
    target_compile_options(remote_surface_test PRIVATE /EHsc)
  endif()
  add_test(NAME remote_surface COMMAND remote_surface_test)
  add_executable(remote_rgb_surface_test remote_rgb_surface_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_scene.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_surface.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_rgb_surface.cpp")
  target_include_directories(remote_rgb_surface_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(remote_rgb_surface_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(remote_rgb_surface_test)
  if(MSVC)
    target_compile_options(remote_rgb_surface_test PRIVATE /EHsc)
  endif()
  add_test(NAME remote_rgb_surface COMMAND remote_rgb_surface_test)
  add_executable(gimbal_control_test gimbal_control_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/gimbal_control.cpp")
  target_include_directories(gimbal_control_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(gimbal_control_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(gimbal_control_test)
  if(MSVC)
    target_compile_options(gimbal_control_test PRIVATE /EHsc)
  endif()
  add_test(NAME gimbal_control COMMAND gimbal_control_test)
  add_executable(gimbal_client_test gimbal_client_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/gimbal_client.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/gimbal_control.cpp")
  target_include_directories(gimbal_client_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(gimbal_client_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(gimbal_client_test)
  if(MSVC)
    target_compile_options(gimbal_client_test PRIVATE /EHsc)
  endif()
  add_test(NAME gimbal_client COMMAND gimbal_client_test)
  # The Python/Java TLS integration test supplies the real wire transcript.
  add_executable(gimbal_interop_test gimbal_interop_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/gimbal_client.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/gimbal_control.cpp")
  target_include_directories(gimbal_interop_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(gimbal_interop_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(gimbal_interop_test)
  if(MSVC)
    target_compile_options(gimbal_interop_test PRIVATE /EHsc)
  endif()
  add_executable(remote_surface_renderer_gl_test remote_surface_renderer_gl_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_scene_renderer.cpp")
  target_compile_definitions(remote_surface_renderer_gl_test PRIVATE __ANDROID__=1)
  target_include_directories(remote_surface_renderer_gl_test PRIVATE
    "${PROJECT_SOURCE_DIR}/tests/remote_renderer_stubs"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp"
    "${PROJECT_SOURCE_DIR}/src/kinematics/include")
  target_compile_features(remote_surface_renderer_gl_test PRIVATE cxx_std_20)
  quest_newton_warnings(remote_surface_renderer_gl_test)
  if(MSVC)
    target_compile_options(remote_surface_renderer_gl_test PRIVATE /EHsc)
  endif()
  add_test(NAME remote_surface_renderer_gl COMMAND remote_surface_renderer_gl_test)
  add_executable(remote_inspection_test remote_inspection_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/remote_inspection.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/arm_renderer.cpp")
  target_include_directories(remote_inspection_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(remote_inspection_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(remote_inspection_test)
  if(MSVC)
    target_compile_options(remote_inspection_test PRIVATE /EHsc)
  endif()
  add_test(NAME remote_inspection COMMAND remote_inspection_test)
  add_library(quest_newton_full_control STATIC
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/full_control.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/interaction_trace.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/registered_input.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/sim_settings.cpp")
  target_include_directories(quest_newton_full_control PUBLIC "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(quest_newton_full_control PUBLIC quest_newton_kinematics)
  quest_newton_warnings(quest_newton_full_control)
  add_executable(interaction_replay "${PROJECT_SOURCE_DIR}/tools/verification/interaction_replay.cpp")
  target_link_libraries(interaction_replay PRIVATE quest_newton_full_control)
  quest_newton_warnings(interaction_replay)
  add_executable(interaction_trace_test interaction_trace_test.cpp)
  target_link_libraries(interaction_trace_test PRIVATE quest_newton_full_control)
  quest_newton_warnings(interaction_trace_test)
  add_test(NAME interaction_trace COMMAND interaction_trace_test)
  add_executable(full_control_test full_control_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/full_control.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/registered_input.cpp")
  target_include_directories(full_control_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(full_control_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(full_control_test)
  if(MSVC)
    target_compile_options(full_control_test PRIVATE /EHsc)
    target_compile_options(quest_newton_full_control PRIVATE /EHsc)
    target_compile_options(interaction_replay PRIVATE /EHsc)
    target_compile_options(interaction_trace_test PRIVATE /EHsc)
  endif()
  add_test(NAME full_control COMMAND full_control_test)
  add_executable(pick_place_test pick_place_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/object_interaction.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/scene_details.cpp")
  target_include_directories(pick_place_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(pick_place_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(pick_place_test)
  if(MSVC)
    target_compile_options(pick_place_test PRIVATE /EHsc)
  endif()
  add_test(NAME pick_place COMMAND pick_place_test)
  add_executable(control_boundary_test control_boundary_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/base_registration.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/registered_input.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/sim_settings.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/room_environment.cpp")
  target_include_directories(control_boundary_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(control_boundary_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(control_boundary_test)
  if(MSVC)
    target_compile_options(control_boundary_test PRIVATE /EHsc)
  endif()
  add_test(NAME control_boundary COMMAND control_boundary_test)
  add_executable(room_environment_test room_environment_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/room_environment.cpp")
  target_include_directories(room_environment_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(room_environment_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(room_environment_test)
  if(MSVC)
    target_compile_options(room_environment_test PRIVATE /EHsc)
  endif()
  add_test(NAME room_environment COMMAND room_environment_test)
  set(QUEST_NEWTON_OPENXR_INCLUDE_DIR "" CACHE PATH "OpenXR headers for host Scene API tests")
  if(QUEST_NEWTON_OPENXR_INCLUDE_DIR)
    add_executable(room_scene_test room_scene_test.cpp
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/room_scene.cpp"
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/room_environment.cpp")
    target_include_directories(room_scene_test PRIVATE
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp" "${QUEST_NEWTON_OPENXR_INCLUDE_DIR}")
    target_link_libraries(room_scene_test PRIVATE quest_newton_kinematics)
    quest_newton_warnings(room_scene_test)
    if(MSVC)
      target_compile_options(room_scene_test PRIVATE /EHsc)
    endif()
    add_test(NAME room_scene COMMAND room_scene_test)
  endif()
  add_executable(full_benchmark_test full_benchmark_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/sim_settings.cpp")
  target_include_directories(full_benchmark_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(full_benchmark_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(full_benchmark_test)
  if(MSVC)
    target_compile_options(full_benchmark_test PRIVATE /EHsc)
  endif()
  add_test(NAME full_benchmark COMMAND full_benchmark_test)
  add_executable(full_scene_test full_scene_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/scene_snapshot.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/registered_input.cpp")
  target_include_directories(full_scene_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(full_scene_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(full_scene_test)
  if(MSVC)
    target_compile_options(full_scene_test PRIVATE /EHsc)
  endif()
  add_test(NAME full_scene COMMAND full_scene_test)
  add_executable(sim_controls_test sim_controls_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/sim_settings.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/base_registration.cpp")
  target_include_directories(sim_controls_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(sim_controls_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(sim_controls_test)
  if(MSVC)
    target_compile_options(sim_controls_test PRIVATE /EHsc)
  endif()
  add_test(NAME sim_controls COMMAND sim_controls_test)
  add_executable(arm_renderer_test arm_renderer_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/arm_renderer.cpp"
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/passthrough_layer.cpp")
  target_include_directories(arm_renderer_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_compile_features(arm_renderer_test PRIVATE cxx_std_20)
  quest_newton_warnings(arm_renderer_test)
  if(MSVC)
    target_compile_options(arm_renderer_test PRIVATE /EHsc)
  endif()
  add_test(NAME arm_renderer COMMAND arm_renderer_test)
  add_executable(newton_runtime_test newton_runtime_test.cpp)
  target_compile_features(newton_runtime_test PRIVATE cxx_std_20)
  target_link_libraries(newton_runtime_test PRIVATE quest_newton_runtime)
  quest_newton_warnings(newton_runtime_test)
  if(MSVC)
    target_compile_options(newton_runtime_test PRIVATE /EHsc)
  endif()

  add_test(NAME newton_runtime COMMAND newton_runtime_test)
  add_executable(franka_kinematics_test franka_kinematics_test.cpp)
  target_link_libraries(franka_kinematics_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(franka_kinematics_test)
  add_test(NAME franka_kinematics COMMAND franka_kinematics_test)
  add_executable(input_mapper_test input_mapper_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/input_mapper.cpp")
  target_include_directories(input_mapper_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(input_mapper_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(input_mapper_test)
  if(MSVC)
    target_compile_options(franka_kinematics_test PRIVATE /EHsc)
    target_compile_options(input_mapper_test PRIVATE /EHsc)
  endif()
  add_test(NAME input_mapper COMMAND input_mapper_test)
  add_executable(joint_target_mailbox_test joint_target_mailbox_test.cpp)
  target_include_directories(joint_target_mailbox_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(joint_target_mailbox_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(joint_target_mailbox_test)
  if(MSVC)
    target_compile_options(joint_target_mailbox_test PRIVATE /EHsc)
  endif()
  add_test(NAME joint_target_mailbox COMMAND joint_target_mailbox_test)
  add_executable(mesh_geometry_test mesh_geometry_test.cpp
    "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/mesh_geometry.cpp")
  target_include_directories(mesh_geometry_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  quest_newton_warnings(mesh_geometry_test)
  if(MSVC)
    target_compile_options(mesh_geometry_test PRIVATE /EHsc)
  endif()
  add_test(NAME mesh_geometry COMMAND mesh_geometry_test)
  add_executable(timing_recorder_test timing_recorder_test.cpp)
  target_include_directories(timing_recorder_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  quest_newton_warnings(timing_recorder_test)
  if(MSVC)
    target_compile_options(timing_recorder_test PRIVATE /EHsc)
  endif()
  add_test(NAME timing_recorder COMMAND timing_recorder_test)
  add_executable(launch_options_test launch_options_test.cpp)
  target_include_directories(launch_options_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  quest_newton_warnings(launch_options_test)
  if(MSVC)
    target_compile_options(launch_options_test PRIVATE /EHsc)
  endif()
  add_test(NAME launch_options COMMAND launch_options_test)
  add_executable(verification_state_limits_test verification_state_limits_test.cpp)
  target_include_directories(verification_state_limits_test PRIVATE "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp")
  target_link_libraries(verification_state_limits_test PRIVATE quest_newton_kinematics)
  quest_newton_warnings(verification_state_limits_test)
  if(MSVC)
    target_compile_options(verification_state_limits_test PRIVATE /EHsc)
  endif()
  add_test(NAME verification_state_limits COMMAND verification_state_limits_test)
  set(QUEST_NEWTON_TRACE_INCLUDE_DIR "" CACHE PATH "Generated recorded-controller trace binding")
  if(QUEST_NEWTON_TRACE_INCLUDE_DIR)
    get_filename_component(trace_include_dir "${QUEST_NEWTON_TRACE_INCLUDE_DIR}" ABSOLUTE
      BASE_DIR "${PROJECT_SOURCE_DIR}")
    add_executable(controller_trace_reference controller_trace_reference.cpp
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/input_mapper.cpp")
    target_include_directories(controller_trace_reference PRIVATE
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp" "${trace_include_dir}")
    target_link_libraries(controller_trace_reference PRIVATE quest_newton_kinematics)
    quest_newton_warnings(controller_trace_reference)
    if(MSVC)
      target_compile_options(controller_trace_reference PRIVATE /EHsc)
    endif()
  endif()
  set(QUEST_NEWTON_MESH_INCLUDE_DIR "" CACHE PATH "Verified generated mesh binding for real visual parity")
  if(QUEST_NEWTON_MESH_INCLUDE_DIR)
    get_filename_component(mesh_include_dir "${QUEST_NEWTON_MESH_INCLUDE_DIR}" ABSOLUTE
      BASE_DIR "${PROJECT_SOURCE_DIR}")
    add_executable(arm_visual_parity_test arm_visual_parity_test.cpp
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp/arm_renderer.cpp")
    target_include_directories(arm_visual_parity_test PRIVATE
      "${PROJECT_SOURCE_DIR}/quest/app/src/main/cpp" "${mesh_include_dir}")
    quest_newton_warnings(arm_visual_parity_test)
    if(MSVC)
      target_compile_options(arm_visual_parity_test PRIVATE /EHsc)
    endif()
    # The real verifier wrapper supplies the Newton/URDF fixture explicitly;
    # this is deliberately not an environment-free CTest with skipped inputs.
  endif()
  find_program(QUEST_NEWTON_PWSH pwsh)
  if(QUEST_NEWTON_PWSH)
    add_test(NAME task5_device_evidence
      COMMAND "${QUEST_NEWTON_PWSH}" -NoProfile -File
        "${CMAKE_CURRENT_SOURCE_DIR}/device/task5_checks_test.ps1")
  endif()
endif()
