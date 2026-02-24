#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>

#include <portaudio.h>

// Dear ImGui
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include "dsp/engines/tune_hall/ReverbEngine.h"
#include "dsp/modes/Modes.h"

// ================================================================================================
// Big Pi — GUI Tuner App (Linux laptop)
//   Real-time audio I/O + GUI sliders + pitch trigger
//
// Keys:
//   t : trigger 1.5s pitch burst
//   p : toggle continuous pitch
//   1..6 : load A/B variants (baseline -> full hybrid)
// ================================================================================================

static constexpr int   kSampleRate = 48000;
static constexpr int   kProcBlock  = 64;
static constexpr float kPi         = 3.14159265358979323846f;

static inline float clampf(float x, float a, float b) {
    return (x < a) ? a : (x > b) ? b : x;
}

// ------------------------------------------------------------------------------------------------
// GUI -> Audio parameter bridge.
// ------------------------------------------------------------------------------------------------
struct AtomicParams {
    // Core
    std::atomic<float> mix{0.35f};
    std::atomic<float> decay{0.94f};
    std::atomic<float> predelayMs{25.0f};
    std::atomic<float> dampingHz{9000.0f};
    std::atomic<float> feedbackHpHz{30.0f};

    // Stereo / output
    std::atomic<float> stereoDepth{1.0f};
    std::atomic<float> outWidth{1.10f};
    std::atomic<float> outLevel{1.0f};

    // Modulation + jitter
    std::atomic<float> modDepthMs{6.0f};
    std::atomic<float> modRateHz{0.20f};

    std::atomic<float> modJitterEnable{1.0f};
    std::atomic<float> modJitterAmount{0.30f};
    std::atomic<float> modJitterRateHz{0.35f};
    std::atomic<float> modJitterSmoothMs{80.0f};

    // Diffusion
    std::atomic<float> lateDiffEnable{1.0f};
    std::atomic<float> lateDiffAmount{0.60f};
    std::atomic<float> lateDiffMinG{0.45f};
    std::atomic<float> lateDiffMaxG{0.72f};

    // ER
    std::atomic<float> erLevel{0.30f};
    std::atomic<float> erSize{0.60f};
    std::atomic<float> erDampHz{9000.0f};
    std::atomic<float> erWidth{1.0f};

    // Multiband decay coloration
    std::atomic<float> fbXoverLoHz{250.0f};
    std::atomic<float> fbXoverHiHz{3500.0f};
    std::atomic<float> decayLowMul{1.08f};
    std::atomic<float> decayMidMul{1.00f};
    std::atomic<float> decayHighMul{0.90f};

    // Cloud toggles + key amounts
    std::atomic<float> cloudEnable{1.0f};
    std::atomic<float> cloudFrontEnable{1.0f};
    std::atomic<float> cloudDelaySetEnable{1.0f};
    std::atomic<float> cloudSmearEnable{1.0f};
    std::atomic<float> dynDiffEnable{1.0f};

    std::atomic<float> cloudFrontAmount{0.50f};
    std::atomic<float> cloudFrontSizeMs{24.0f};
    std::atomic<float> cloudFrontWidth{0.75f};

    std::atomic<float> cloudSmearAmount{0.34f};
    std::atomic<float> cloudSmearTimeMs{14.0f};
    std::atomic<float> cloudSmearWidth{0.80f};
};

static void applyParams(ReverbEngine::Params& p, const AtomicParams& a) {
    // Default to Sky for cloud-ish testing (change later via dropdown if desired)
    p.mode = bigpi::Mode::Sky;

    p.mix = clampf(a.mix.load(), 0.0f, 1.0f);
    p.decay = clampf(a.decay.load(), 0.0f, 0.9995f);
    p.predelayMs = clampf(a.predelayMs.load(), 0.0f, 200.0f);
    p.dampingHz = clampf(a.dampingHz.load(), 500.0f, 20000.0f);
    p.feedbackHpHz = clampf(a.feedbackHpHz.load(), 0.0f, 400.0f);

    p.stereoDepth = clampf(a.stereoDepth.load(), 0.0f, 1.0f);
    p.outWidth = clampf(a.outWidth.load(), 0.0f, 2.0f);
    p.outLevel = clampf(a.outLevel.load(), 0.0f, 2.0f);

    p.modDepthMs = clampf(a.modDepthMs.load(), 0.0f, 20.0f);
    p.modRateHz = clampf(a.modRateHz.load(), 0.0f, 5.0f);

    p.modJitterEnable = clampf(a.modJitterEnable.load(), 0.0f, 1.0f);
    p.modJitterAmount = clampf(a.modJitterAmount.load(), 0.0f, 2.0f);
    p.modJitterRateHz = clampf(a.modJitterRateHz.load(), 0.0f, 5.0f);
    p.modJitterSmoothMs = clampf(a.modJitterSmoothMs.load(), 0.0f, 500.0f);

    p.lateDiffEnable = clampf(a.lateDiffEnable.load(), 0.0f, 1.0f);
    p.lateDiffAmount = clampf(a.lateDiffAmount.load(), 0.0f, 1.0f);
    p.lateDiffMinG = clampf(a.lateDiffMinG.load(), 0.0f, 0.999f);
    p.lateDiffMaxG = clampf(a.lateDiffMaxG.load(), 0.0f, 0.999f);
    if (p.lateDiffMinG > p.lateDiffMaxG) std::swap(p.lateDiffMinG, p.lateDiffMaxG);

    p.erLevel = clampf(a.erLevel.load(), 0.0f, 1.0f);
    p.erSize = clampf(a.erSize.load(), 0.0f, 1.0f);
    p.erDampHz = clampf(a.erDampHz.load(), 500.0f, 20000.0f);
    p.erWidth = clampf(a.erWidth.load(), 0.0f, 2.0f);

    p.fbXoverLoHz = clampf(a.fbXoverLoHz.load(), 50.0f, 2000.0f);
    p.fbXoverHiHz = clampf(a.fbXoverHiHz.load(), 500.0f, 12000.0f);
    p.decayLowMul = clampf(a.decayLowMul.load(), 0.2f, 2.0f);
    p.decayMidMul = clampf(a.decayMidMul.load(), 0.2f, 2.0f);
    p.decayHighMul = clampf(a.decayHighMul.load(), 0.2f, 2.0f);

    p.cloudEnable = clampf(a.cloudEnable.load(), 0.0f, 1.0f);
    p.cloudFrontEnable = clampf(a.cloudFrontEnable.load(), 0.0f, 1.0f);
    p.cloudDelaySetEnable = clampf(a.cloudDelaySetEnable.load(), 0.0f, 1.0f);
    p.cloudSmearEnable = clampf(a.cloudSmearEnable.load(), 0.0f, 1.0f);
    p.dynDiffEnable = clampf(a.dynDiffEnable.load(), 0.0f, 1.0f);

    p.cloudFrontAmount = clampf(a.cloudFrontAmount.load(), 0.0f, 1.0f);
    p.cloudFrontSizeMs = clampf(a.cloudFrontSizeMs.load(), 0.0f, 100.0f);
    p.cloudFrontWidth = clampf(a.cloudFrontWidth.load(), 0.0f, 2.0f);

    p.cloudSmearAmount = clampf(a.cloudSmearAmount.load(), 0.0f, 1.0f);
    p.cloudSmearTimeMs = clampf(a.cloudSmearTimeMs.load(), 0.0f, 100.0f);
    p.cloudSmearWidth = clampf(a.cloudSmearWidth.load(), 0.0f, 2.0f);
}

static void loadVariant(int v, AtomicParams& a) {
    // Baseline: everything cloud off + no stereo depth.
    auto baseline = [&]() {
        a.stereoDepth.store(0.0f);
        a.cloudEnable.store(0.0f);
        a.cloudFrontEnable.store(0.0f);
        a.cloudDelaySetEnable.store(0.0f);
        a.cloudSmearEnable.store(0.0f);
        a.dynDiffEnable.store(0.0f);
    };

    baseline();

    switch (v) {
        default:
        case 1: break; // baseline
        case 2: a.stereoDepth.store(1.0f); break; // stereo only
        case 3: a.cloudEnable.store(1.0f); break; // main cloudify enable
        case 4: a.cloudFrontEnable.store(1.0f); break; // front spray only
        case 5:
            a.cloudDelaySetEnable.store(1.0f);
            a.cloudSmearEnable.store(1.0f);
            break;
        case 6: // full hybrid
            a.stereoDepth.store(1.0f);
            a.cloudEnable.store(1.0f);
            a.cloudFrontEnable.store(1.0f);
            a.cloudDelaySetEnable.store(1.0f);
            a.cloudSmearEnable.store(1.0f);
            a.dynDiffEnable.store(1.0f);
            break;
    }
}

struct AudioState {
    ReverbEngine reverb;
    ReverbEngine::Params params{};
    AtomicParams* atomics{nullptr};

    // Pitch injection
    std::atomic<bool> continuousPitch{false};
    std::atomic<int>  pitchBurstSamps{0};
    float pitchFreqHz{440.0f};
    float pitchAmp{0.20f};
    float phase{0.0f};

    // Metering
    std::atomic<float> meterInPeak{0.0f};
    std::atomic<float> meterOutPeak{0.0f};
};

static int paCallback(const void* input, void* output,
                      unsigned long frameCount,
                      const PaStreamCallbackTimeInfo*,
                      PaStreamCallbackFlags,
                      void* userData)
{
    auto* st = static_cast<AudioState*>(userData);
    float* out = static_cast<float*>(output);
    const float* in = static_cast<const float*>(input);

    float inPeak = 0.0f;
    float outPeak = 0.0f;

    const float w = 2.0f * kPi * st->pitchFreqHz / (float)kSampleRate;

    unsigned long done = 0;
    while (done < frameCount) {
        const unsigned long n = std::min<unsigned long>(kProcBlock, frameCount - done);

        // Copy GUI params once per chunk
        applyParams(st->params, *st->atomics);
        st->reverb.setParams(st->params);

        float inL[kProcBlock]{};
        float inR[kProcBlock]{};
        float outL[kProcBlock]{};
        float outR[kProcBlock]{};

        for (unsigned long i = 0; i < n; ++i) {
            const unsigned long idx = (done + i) * 2;

            float xL = in ? in[idx + 0] : 0.0f;
            float xR = in ? in[idx + 1] : 0.0f;

            int burst = st->pitchBurstSamps.load(std::memory_order_relaxed);
            const bool cont = st->continuousPitch.load(std::memory_order_relaxed);

            float add = 0.0f;
            if (cont || burst > 0) {
                add = st->pitchAmp * std::sin(st->phase);
                st->phase += w;
                if (st->phase > 2.0f * kPi) st->phase -= 2.0f * kPi;
                if (!cont && burst > 0) burst--;
            }
            if (!cont) st->pitchBurstSamps.store(burst, std::memory_order_relaxed);

            xL += add;
            xR += add;

            inL[i] = xL;
            inR[i] = xR;

            inPeak = std::max(inPeak, std::max(std::fabs(xL), std::fabs(xR)));
        }

        st->reverb.processBlock(inL, inR, outL, outR, (int)n);

        for (unsigned long i = 0; i < n; ++i) {
            outPeak = std::max(outPeak, std::max(std::fabs(outL[i]), std::fabs(outR[i])));
            const unsigned long idx = (done + i) * 2;
            out[idx + 0] = outL[i];
            out[idx + 1] = outR[i];
        }

        done += n;
    }

    st->meterInPeak.store(inPeak, std::memory_order_relaxed);
    st->meterOutPeak.store(outPeak, std::memory_order_relaxed);

    return paContinue;
}

static void uiHelp() {
    ImGui::TextUnformatted("Keys:");
    ImGui::BulletText("t: trigger 1.5s pitch burst");
    ImGui::BulletText("p: toggle continuous pitch");
    ImGui::BulletText("1..6: load A/B variants");
    ImGui::BulletText("Close window to quit");
}

int main() {
    // --- PortAudio ---
    if (Pa_Initialize() != paNoError) {
        std::cerr << "PortAudio init failed.\n";
        return 1;
    }

    // --- GLFW ---
    if (!glfwInit()) {
        std::cerr << "GLFW init failed.\n";
        Pa_Terminate();
        return 1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1080, 720, "Big Pi — GUI Tuner", nullptr, nullptr);
    if (!window) {
        std::cerr << "GLFW window create failed.\n";
        glfwTerminate();
        Pa_Terminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // --- ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // --- Engine state ---
    AtomicParams atomics;
    loadVariant(6, atomics); // start full hybrid

    AudioState st;
    st.atomics = &atomics;
    st.reverb.prepare((float)kSampleRate, kProcBlock);

    // --- Audio stream ---
    PaStream* stream = nullptr;

    PaStreamParameters inParams{};
    inParams.device = Pa_GetDefaultInputDevice();
    inParams.channelCount = 2;
    inParams.sampleFormat = paFloat32;
    if (inParams.device != paNoDevice) {
        inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultLowInputLatency;
    }

    PaStreamParameters outParams{};
    outParams.device = Pa_GetDefaultOutputDevice();
    outParams.channelCount = 2;
    outParams.sampleFormat = paFloat32;
    outParams.suggestedLatency = Pa_GetDeviceInfo(outParams.device)->defaultLowOutputLatency;

    PaError err;
    if (inParams.device == paNoDevice) {
        std::cerr << "No default input device found (running with silence input).\n";
        err = Pa_OpenStream(&stream, nullptr, &outParams, kSampleRate, kProcBlock, paNoFlag, paCallback, &st);
    } else {
        err = Pa_OpenStream(&stream, &inParams, &outParams, kSampleRate, kProcBlock, paNoFlag, paCallback, &st);
    }

    if (err != paNoError) {
        std::cerr << "Pa_OpenStream failed: " << Pa_GetErrorText(err) << "\n";
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        Pa_Terminate();
        return 1;
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream failed: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream);
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        Pa_Terminate();
        return 1;
    }

    bool showHelp = true;

    // --- Main UI loop ---
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Hotkeys
        if (ImGui::IsKeyPressed(ImGuiKey_T, false)) st.pitchBurstSamps.store((int)(1.5f * kSampleRate));
        if (ImGui::IsKeyPressed(ImGuiKey_P, false)) st.continuousPitch.store(!st.continuousPitch.load());

        if (ImGui::IsKeyPressed(ImGuiKey_1, false)) { loadVariant(1, atomics); st.reverb.reset(); }
        if (ImGui::IsKeyPressed(ImGuiKey_2, false)) { loadVariant(2, atomics); st.reverb.reset(); }
        if (ImGui::IsKeyPressed(ImGuiKey_3, false)) { loadVariant(3, atomics); st.reverb.reset(); }
        if (ImGui::IsKeyPressed(ImGuiKey_4, false)) { loadVariant(4, atomics); st.reverb.reset(); }
        if (ImGui::IsKeyPressed(ImGuiKey_5, false)) { loadVariant(5, atomics); st.reverb.reset(); }
        if (ImGui::IsKeyPressed(ImGuiKey_6, false)) { loadVariant(6, atomics); st.reverb.reset(); }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Big Pi — Tuner");

        if (ImGui::Button("Trigger Pitch (t)")) st.pitchBurstSamps.store((int)(1.5f * kSampleRate));
        ImGui::SameLine();
        bool cont = st.continuousPitch.load();
        if (ImGui::Checkbox("Continuous pitch (p)", &cont)) st.continuousPitch.store(cont);
        ImGui::SameLine();
        ImGui::Checkbox("Help", &showHelp);

        ImGui::Separator();
        ImGui::Text("Input peak:  %.3f", st.meterInPeak.load());
        ImGui::SameLine();
        ImGui::Text("Output peak: %.3f", st.meterOutPeak.load());

        ImGui::Separator();
        ImGui::TextUnformatted("Variants:");
        if (ImGui::Button("1 Baseline")) { loadVariant(1, atomics); st.reverb.reset(); }
        ImGui::SameLine();
        if (ImGui::Button("2 Stereo")) { loadVariant(2, atomics); st.reverb.reset(); }
        ImGui::SameLine();
        if (ImGui::Button("3 Cloudify")) { loadVariant(3, atomics); st.reverb.reset(); }
        ImGui::SameLine();
        if (ImGui::Button("4 Front")) { loadVariant(4, atomics); st.reverb.reset(); }
        ImGui::SameLine();
        if (ImGui::Button("5 Delay+Smear")) { loadVariant(5, atomics); st.reverb.reset(); }
        ImGui::SameLine();
        if (ImGui::Button("6 Full")) { loadVariant(6, atomics); st.reverb.reset(); }

        ImGui::Separator();

        auto slider = [](const char* label, std::atomic<float>& a, float lo, float hi) {
            float v = a.load();
            if (ImGui::SliderFloat(label, &v, lo, hi)) a.store(v);
        };

        if (ImGui::CollapsingHeader("Core", ImGuiTreeNodeFlags_DefaultOpen)) {
            slider("Mix", atomics.mix, 0.0f, 1.0f);
            slider("Decay", atomics.decay, 0.0f, 0.9995f);
            slider("PreDelay (ms)", atomics.predelayMs, 0.0f, 200.0f);
            slider("Damping (Hz)", atomics.dampingHz, 500.0f, 20000.0f);
            slider("Feedback HP (Hz)", atomics.feedbackHpHz, 0.0f, 400.0f);
        }

        if (ImGui::CollapsingHeader("Stereo + Output", ImGuiTreeNodeFlags_DefaultOpen)) {
            slider("StereoDepth", atomics.stereoDepth, 0.0f, 1.0f);
            slider("Out Width", atomics.outWidth, 0.0f, 2.0f);
            slider("Out Level", atomics.outLevel, 0.0f, 2.0f);
        }

        if (ImGui::CollapsingHeader("Modulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            slider("Mod Depth (ms)", atomics.modDepthMs, 0.0f, 20.0f);
            slider("Mod Rate (Hz)", atomics.modRateHz, 0.0f, 5.0f);
            slider("Jitter Enable", atomics.modJitterEnable, 0.0f, 1.0f);
            slider("Jitter Amount", atomics.modJitterAmount, 0.0f, 2.0f);
            slider("Jitter Rate (Hz)", atomics.modJitterRateHz, 0.0f, 5.0f);
            slider("Jitter Smooth (ms)", atomics.modJitterSmoothMs, 0.0f, 500.0f);
        }

        if (ImGui::CollapsingHeader("Diffusion", ImGuiTreeNodeFlags_DefaultOpen)) {
            slider("LateDiff Enable", atomics.lateDiffEnable, 0.0f, 1.0f);
            slider("LateDiff Amount", atomics.lateDiffAmount, 0.0f, 1.0f);
            slider("LateDiff MinG", atomics.lateDiffMinG, 0.0f, 0.999f);
            slider("LateDiff MaxG", atomics.lateDiffMaxG, 0.0f, 0.999f);
            slider("DynDiff Enable", atomics.dynDiffEnable, 0.0f, 1.0f);
        }

        if (ImGui::CollapsingHeader("Early Reflections")) {
            slider("ER Level", atomics.erLevel, 0.0f, 1.0f);
            slider("ER Size", atomics.erSize, 0.0f, 1.0f);
            slider("ER Damping (Hz)", atomics.erDampHz, 500.0f, 20000.0f);
            slider("ER Width", atomics.erWidth, 0.0f, 2.0f);
        }

        if (ImGui::CollapsingHeader("Multiband Decay")) {
            slider("Xover Lo (Hz)", atomics.fbXoverLoHz, 50.0f, 2000.0f);
            slider("Xover Hi (Hz)", atomics.fbXoverHiHz, 500.0f, 12000.0f);
            slider("Decay Low Mul", atomics.decayLowMul, 0.2f, 2.0f);
            slider("Decay Mid Mul", atomics.decayMidMul, 0.2f, 2.0f);
            slider("Decay High Mul", atomics.decayHighMul, 0.2f, 2.0f);
        }

        if (ImGui::CollapsingHeader("Cloud", ImGuiTreeNodeFlags_DefaultOpen)) {
            slider("Cloud Enable", atomics.cloudEnable, 0.0f, 1.0f);
            slider("Front Enable", atomics.cloudFrontEnable, 0.0f, 1.0f);
            slider("DelaySet Enable", atomics.cloudDelaySetEnable, 0.0f, 1.0f);
            slider("Smear Enable", atomics.cloudSmearEnable, 0.0f, 1.0f);

            slider("Front Amount", atomics.cloudFrontAmount, 0.0f, 1.0f);
            slider("Front Size (ms)", atomics.cloudFrontSizeMs, 0.0f, 100.0f);
            slider("Front Width", atomics.cloudFrontWidth, 0.0f, 2.0f);

            slider("Smear Amount", atomics.cloudSmearAmount, 0.0f, 1.0f);
            slider("Smear Time (ms)", atomics.cloudSmearTimeMs, 0.0f, 100.0f);
            slider("Smear Width", atomics.cloudSmearWidth, 0.0f, 2.0f);
        }

        if (showHelp) {
            ImGui::Separator();
            uiHelp();
        }

        ImGui::End();

        ImGui::Render();
        int display_w = 0, display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // --- Shutdown ---
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
