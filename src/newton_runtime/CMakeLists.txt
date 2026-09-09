add_library(quest_newton_runtime STATIC
  src/dynamic_library.cpp
  src/newton_runtime.cpp
)

target_compile_features(quest_newton_runtime PUBLIC cxx_std_20)
target_include_directories(quest_newton_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
quest_newton_warnings(quest_newton_runtime)

if(MSVC)
  target_compile_options(quest_newton_runtime PRIVATE /EHsc)
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(quest_newton_runtime PRIVATE dl)
endif()
