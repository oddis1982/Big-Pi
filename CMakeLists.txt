cmake_minimum_required(VERSION 3.16)

# Project name
project(BigPi LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If the user didn't specify a build type, default to Debug (helpful for beginners)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Put the compiled executable in build/bin/ (more beginner-friendly)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ------------------------------------------------------------------------------
# Collect source files
# ------------------------------------------------------------------------------

set(SOURCES
    apps/pedal_host/main.cpp

    src/dsp/diffusion/Diffusion.cpp

    src/dsp/engines/tune_hall/EarlyReflections.cpp
    src/dsp/engines/tune_hall/OutputStage.cpp
    src/dsp/engines/tune_hall/ReverbEngine.cpp

    src/dsp/modes/ModePresets.cpp

    src/dsp/tail/Matrices.cpp
    src/dsp/tail/Tank.cpp
    src/dsp/tail/TapPatterns.cpp
)

# ------------------------------------------------------------------------------
# Create executable
# ------------------------------------------------------------------------------

add_executable(bigpi_test ${SOURCES})

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------

# This allows includes like:
#   #include "core/Version.h"
#   #include "dsp/..."
target_include_directories(bigpi_test PRIVATE
    src
)

# ------------------------------------------------------------------------------
# Warnings (super helpful while you're learning)
# ------------------------------------------------------------------------------

if(MSVC)
  target_compile_options(bigpi_test PRIVATE /W4)
else()
  target_compile_options(bigpi_test PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- Append this block to the bottom of your top-level CMakeLists.txt ----
# GUI Tuner (PortAudio + Dear ImGui + GLFW + OpenGL)

set(IMGUI_DIR apps/tuner_gui/third_party/imgui)

add_executable(bigpi_tuner_gui
    apps/tuner_gui/main.cpp

    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp

    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp

    # Big Pi DSP core sources (match your bigpi_test set; add/remove if your tree differs)
    src/dsp/diffusion/Diffusion.cpp

    src/dsp/engines/tune_hall/EarlyReflections.cpp
    src/dsp/engines/tune_hall/OutputStage.cpp
    src/dsp/engines/tune_hall/ReverbEngine.cpp

    src/dsp/modes/ModePresets.cpp

    src/dsp/tail/Matrices.cpp
    src/dsp/tail/Tank.cpp
    src/dsp/tail/TapPatterns.cpp
)

target_include_directories(bigpi_tuner_gui PRIVATE
    src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

target_link_libraries(bigpi_tuner_gui PRIVATE
    OpenGL::GL
    glfw
    ${PORTAUDIO_LIBRARIES}
)

set_target_properties(bigpi_tuner_gui PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

