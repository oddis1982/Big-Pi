#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <algorithm>
#include <string>

#include "core/Version.h"
#include "dsp/engines/tune_hall/ReverbEngine.h"
#include "dsp/modes/Modes.h"

/*
  =============================================================================
  App/main.cpp — Big Pi Test Harness (modular project)
  =============================================================================

  What this program does:
    - Generates a test input signal (impulse or tone burst)
    - Runs it through ReverbEngine
    - Writes the result to a stereo 16-bit WAV file

  Why this matters:
    Reverbs are hard to debug by reading code.
    Listening to impulse responses and tone bursts is a standard technique.

  How to use:
    - Build and run
    - Find the WAV file in your CURRENT working directory
      (usually the build folder you ran the program from)
    - Listen in a DAW or audio player
*/

// ============================================================================
// WAV writer helpers (16-bit PCM stereo)
// ============================================================================

/*
===============================================================================
HOW TO BUILD + RUN THIS TEST HARNESS (BEGINNER GUIDE)
===============================================================================

This program:
  - Generates a test input (impulse or tone burst)
  - Runs it through the reverb
  - Writes a WAV file you can listen to

It does NOT record your guitar or use real audio I/O yet. It is an offline test.

-------------------------------------------------------------------------------
1) PREREQUISITES (Linux)
-------------------------------------------------------------------------------

You need:
  - A C++ compiler (g++)
  - CMake (to configure builds)
  - Make or Ninja (to compile)

On Debian/Ubuntu/Raspberry Pi OS you can install these with:

  sudo apt update
  sudo apt install -y build-essential cmake

Optional but helpful:
  sudo apt install -y ninja-build

-------------------------------------------------------------------------------
2) BUILD (standard CMake workflow)
-------------------------------------------------------------------------------

From the project root (the folder with CMakeLists.txt):

  mkdir -p build
  cd build
  cmake ..
  cmake --build .

-------------------------------------------------------------------------------
3) RUN
-------------------------------------------------------------------------------

Depending on your CMakeLists, the program will usually be here:

  ./bin/bigpi_test

When it runs, it prints the WAV filename it wrote.

The WAV file is written into your CURRENT WORKING DIRECTORY.
If you are inside "build", the WAV will be inside "build".

-------------------------------------------------------------------------------
4) LISTEN
-------------------------------------------------------------------------------

From inside the build folder:

  ls *.wav

To play with ffplay:
  sudo apt install -y ffmpeg
  ffplay your_file.wav

To play with aplay:
  aplay your_file.wav

===============================================================================
END OF BEGINNER GUIDE
===============================================================================
*/

static void write_u32_le(std::ofstream& f, uint32_t v) {
    char b[4];
    b[0] = char(v & 0xFF);
    b[1] = char((v >> 8) & 0xFF);
    b[2] = char((v >> 16) & 0xFF);
    b[3] = char((v >> 24) & 0xFF);
    f.write(b, 4);
}

static void write_u16_le(std::ofstream& f, uint16_t v) {
    char b[2];
    b[0] = char(v & 0xFF);
    b[1] = char((v >> 8) & 0xFF);
    f.write(b, 2);
}

static int16_t float_to_i16(float x) {
    x = std::max(-1.0f, std::min(1.0f, x));
    int v = int(std::lround(x * 32767.0f));
    v = std::max(-32768, std::min(32767, v));
    return (int16_t)v;
}

static bool write_wav_stereo_16(
    const std::string& path,
    const std::vector<float>& L,
    const std::vector<float>& R,
    int sampleRate
) {
    if (L.size() != R.size() || L.empty()) {
        std::cerr << "WAV write failed: invalid channel sizes\n";
        return false;
    }

    const uint16_t numChannels = 2;
    const uint16_t bitsPerSample = 16;
    const uint16_t bytesPerSample = bitsPerSample / 8;
    const uint32_t byteRate = sampleRate * numChannels * bytesPerSample;
    const uint16_t blockAlign = numChannels * bytesPerSample;

    const uint32_t dataBytes = uint32_t(L.size()) * numChannels * bytesPerSample;

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "WAV write failed: cannot open " << path << "\n";
        return false;
    }

    // RIFF
    f.write("RIFF", 4);
    write_u32_le(f, 36 + dataBytes);
    f.write("WAVE", 4);

    // fmt
    f.write("fmt ", 4);
    write_u32_le(f, 16);
    write_u16_le(f, 1); // PCM
    write_u16_le(f, numChannels);
    write_u32_le(f, (uint32_t)sampleRate);
    write_u32_le(f, byteRate);
    write_u16_le(f, blockAlign);
    write_u16_le(f, bitsPerSample);

    // data
    f.write("data", 4);
    write_u32_le(f, dataBytes);

    for (size_t i = 0; i < L.size(); ++i) {
        int16_t li = float_to_i16(L[i]);
        int16_t ri = float_to_i16(R[i]);
        f.write(reinterpret_cast<const char*>(&li), 2);
        f.write(reinterpret_cast<const char*>(&ri), 2);
    }

    return true;
}

// ============================================================================
// Test signals
// ============================================================================

static void generate_impulse(std::vector<float>& L, std::vector<float>& R) {
    std::fill(L.begin(), L.end(), 0.0f);
    std::fill(R.begin(), R.end(), 0.0f);
    if (!L.empty()) {
        L[0] = 1.0f;
        R[0] = 1.0f;
    }
}

static void generate_tone_burst(std::vector<float>& L, std::vector<float>& R,
    int sampleRate,
    float freqHz,
    float burstSeconds,
    float amplitude)
{
    std::fill(L.begin(), L.end(), 0.0f);
    std::fill(R.begin(), R.end(), 0.0f);

    const int burstSamples = std::min<int>((int)L.size(), int(burstSeconds * sampleRate));
    const float w = 2.0f * float(M_PI) * freqHz / float(sampleRate);

    // Fade helps avoid clicks at the start/end of the burst.
    const int fadeSamples = std::max(1, std::min(256, burstSamples / 4));

    for (int n = 0; n < burstSamples; ++n) {
        float env = 1.0f;

        // Fade in
        if (n < fadeSamples) {
            env *= float(n) / float(fadeSamples);
        }

        // Fade out (safe even when fadeSamples == 1)
        const int fadeStart = burstSamples - fadeSamples;
        if (n >= fadeStart) {
            const int idx = (burstSamples - 1) - n;               // counts down to 0
            env *= float(idx) / float(std::max(1, fadeSamples - 1));
        }

        const float s = std::sin(w * float(n));
        const float x = amplitude * env * s;

        L[n] = x;
        R[n] = x;
    }
}

// Helper: build a nice filename so you don’t overwrite old tests.
static std::string make_wav_name(bigpi::Mode mode, bool impulse, bool tone) {
    std::string modeName = bigpi::modeToString(mode);
    std::string sig = impulse ? "impulse" : (tone ? "tone" : "silence");
    return "big_pi_" + modeName + "_" + sig + ".wav";
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "Big Pi — Modular Reverb Test Harness\n";
    std::cout << "Build: " << REVERB_BUILD_ID << "\n\n";

    const int sampleRate = 48000;
    const int seconds = 8;
    const int numSamples = sampleRate * seconds;
    const int blockSize = 64;

    std::vector<float> inL(numSamples), inR(numSamples);
    std::vector<float> outL(numSamples), outR(numSamples);

    // Choose a test input (BEGINNER TIP: set ONE of these true)
    bool doImpulse = true;
    bool doToneBurst = false;

    if (doImpulse && doToneBurst) {
        std::cout << "WARNING: doImpulse and doToneBurst are both true. Using impulse.\n";
        doToneBurst = false;
    }

    if (doImpulse) {
        std::cout << "Generating impulse...\n";
        generate_impulse(inL, inR);
    }
    else if (doToneBurst) {
        std::cout << "Generating tone burst...\n";
        generate_tone_burst(inL, inR, sampleRate, 440.0f, 0.15f, 0.8f);
    }
    else {
        std::cout << "Generating silence...\n";
        std::fill(inL.begin(), inL.end(), 0.0f);
        std::fill(inR.begin(), inR.end(), 0.0f);
    }

    // Create and configure the engine
    ReverbEngine reverb;
    reverb.prepare((float)sampleRate, blockSize);
    reverb.reset();

    ReverbEngine::Params p;

    // Pick a mode to audition:
    p.mode = bigpi::Mode::Hall;  // try Room/Hall/Cathedral/Plate/Vintage etc.

    // A few safety clamps for beginners (prevents runaway tails)
    p.mix = std::max(0.0f, std::min(1.0f, 0.35f));
    p.decay = std::max(0.0f, std::min(0.9995f, 0.94f));

    // Core controls
    p.predelayMs = 25.0f;
    p.dampingHz = 9000.0f;
    p.feedbackHpHz = 30.0f;

    // Modulation
    p.modDepthMs = 7.0f;
    p.modRateHz = 0.20f;

    p.modJitterEnable = 1.0f;
    p.modJitterAmount = 0.35f;
    p.modJitterRateHz = 0.35f;
    p.modJitterSmoothMs = 80.0f;

    // Early reflections
    p.erLevel = 0.30f;
    p.erSize = 0.60f;
    p.erDampHz = 9000.0f;
    p.erWidth = 1.0f;

    // Multiband decay coloration
    p.fbXoverLoHz = 250.0f;
    p.fbXoverHiHz = 3500.0f;
    p.decayLowMul = 1.08f;
    p.decayMidMul = 1.00f;
    p.decayHighMul = 0.90f;

    // Late diffusion refinement
    p.lateDiffEnable = 1.0f;
    p.lateDiffAmount = 0.60f;
    p.lateDiffMinG = 0.45f;
    p.lateDiffMaxG = 0.72f;

    // Output stage
    p.outWidth = 1.10f;
    p.outHpHz = 20.0f;
    p.outLowShelfHz = 200.0f;
    p.outLowGainDb = 0.0f;
    p.outHighShelfHz = 8000.0f;
    p.outHighGainDb = 0.0f;
    p.outDrive = 0.0f;
    p.outLevel = 1.0f;

    // Dynamics options
    p.freeze = 0.0f;

    p.duckEnable = 0.0f;
    p.duckThresholdDb = -28.0f;
    p.duckDepthDb = 10.0f;

    p.loudCompEnable = 1.0f;
    p.loudCompStrength = 0.50f;
    p.loudCompMaxDb = 9.0f;

    reverb.setParams(p);

    // Process in blocks
    std::cout << "Processing...\n";

    for (int pos = 0; pos < numSamples; pos += blockSize) {
        int n = std::min(blockSize, numSamples - pos);

        reverb.processBlock(
            &inL[pos], &inR[pos],
            &outL[pos], &outR[pos],
            n
        );
    }

    // Write result (mode + signal type in filename)
    const std::string wavName = make_wav_name(p.mode, doImpulse, doToneBurst);

    if (write_wav_stereo_16(wavName, outL, outR, sampleRate)) {
        std::cout << "Wrote WAV: " << wavName << "\n";
        std::cout << "Tip: The WAV is in the folder you ran the program from.\n";
        std::cout << "Try switching p.mode and re-running to compare modes.\n";
    }
    else {
        std::cerr << "Failed to write WAV.\n";
        return 1;
    }

    std::cout << "Done.\n";
    return 0;
}

