Big Pi — GUI Tuner App (Linux laptop)
===================================

This app provides:
  - Real-time audio I/O via PortAudio
  - Real-time parameter tweaking via Dear ImGui + GLFW + OpenGL3
  - Key-triggered pitch burst (press 't') to quickly diagnose metallic buzz / modulation artifacts

Keys:
  t : trigger 1.5s pitch burst into the input
  p : toggle continuous pitch
  1..6 : load A/B variants (baseline..full hybrid)
  Close window to quit

--------------------------------------------------------------------------------
Dependencies (Ubuntu / Debian)
--------------------------------------------------------------------------------

sudo apt update
sudo apt install -y build-essential cmake pkg-config \
  libportaudio2 portaudio19-dev \
  libglfw3-dev libgl1-mesa-dev

--------------------------------------------------------------------------------
Dear ImGui (you must add it to the repo)
--------------------------------------------------------------------------------

From repo root:

  mkdir -p apps/tuner_gui/third_party
  git clone https://github.com/ocornut/imgui.git apps/tuner_gui/third_party/imgui

The build expects:
  apps/tuner_gui/third_party/imgui/imgui.h
  apps/tuner_gui/third_party/imgui/backends/imgui_impl_glfw.h
  apps/tuner_gui/third_party/imgui/backends/imgui_impl_opengl3.h

--------------------------------------------------------------------------------
Build + run
--------------------------------------------------------------------------------

cmake -S . -B build
cmake --build build -j

Run:
  ./build/bin/bigpi_tuner_gui
