// Copyright (C) 2026 Ryan Voss
//
// This file is part of Omega Modules for VCV Rack 2.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

/*
 * Dynamics.cpp  –  Multiband Upward/Downward Compressor for VCV Rack 2
 * Signal path:
 *   Input  →  Stereo L/R processed together using SSE
 *          →  [IN gain]
 *          →  LR4 crossover  →  LOW / MID / HIGH bands
 *          →  Each band: envelope follower → dual compressor (down + up) → band gain
 *          →  Sum bands
 *          →  MIX wet/dry blend with dry signal
 *          →  [OUT gain]  →  Output
 *
 * Crossover:   LR4 (Linkwitz-Riley 4th order, two cascaded Butterworth 2nd-order)
 *   f1 = 88.3 Hz   (LOW / MID boundary)
 *   f2 = 2500 Hz  (MID / HIGH boundary)
 *
 * Compressor:
 *   MIX     : wet/dry mix  (0 = bypass, 1 = full effect)
 *   TIME    : scale factor applied to all attack/release times  (0.1× – 10×)
 *   IN/OUT  : master gain (±24 dB)
 *   DOWN    : downward compression per band (0 - 200%)
 *   UP      : upward compression per band (0 - 200%)
 *   L/M/H   : band gains (±24 dB), band thresh (-70dB - 0dB)
 */

#include "plugin.hpp"
#include "Maths.hpp"
#include "ui/Components.hpp"
#include "ui/ParamQuantities.hpp"
#include "dsp/Oversample.hpp"
#include "dsp/EnvFollower.hpp"
#include "dsp/Biquad.hpp"

namespace Omega {

constexpr const int64_t PARAM_INTERVAL = 127; // power of 2 - 1
constexpr const int64_t BYPASS_INTERVAL = 1023; // power of 2 - 1
constexpr const float DB_MIN = -70.f;   // bottom of scale
constexpr const float DB_MAX =   0.f;   // top of scale
constexpr const float DB_DELTA[3] = {2.65f, 5.8f, 4.5f}; // band threshold widths/2

struct Band {

    dsp::EnvFollower env;

    __m128d output = _mm_set1_pd(0.0); // stereo L/R
    float indB   = DB_MIN;
    float outdB  = DB_MIN;
    float gaindB = 0.f;
    float downdB = -30.f;
    float updB   = -35.f;
    float downOn = 1.f;
    float upOn   = 1.f;
};

// ─────────────────────────────────────────────────────────────
//    Dynamics Module
// ─────────────────────────────────────────────────────────────
struct Dynamics : Module {

    PanelTheme panelTheme = PanelTheme::AUTO;

    // ── Parameter / Port IDs ────────────────────────────────
    enum ParamId {
        IN_PARAM, OUT_PARAM,
        MIX_PARAM, TIME_PARAM,
        DOWN_PARAM, UP_PARAM,

        // Bands: 0 = HIGH  1 = MID  2 = LOW
        GAIN_PARAM_0, GAIN_PARAM_1, GAIN_PARAM_2,
        THRESH_PARAM_0, THRESH_PARAM_1, THRESH_PARAM_2,
        DOWN_ACTIVE_PARAM_0, DOWN_ACTIVE_PARAM_1, DOWN_ACTIVE_PARAM_2,
        UP_ACTIVE_PARAM_0, UP_ACTIVE_PARAM_1, UP_ACTIVE_PARAM_2,

        LOW_CUTOFF_PARAM, HIGH_CUTOFF_PARAM,
        HARD_CLIP_PARAM,

        PARAMS_LEN
    };
    enum InputId  { LEFT_INPUT,  RIGHT_INPUT, INPUTS_LEN  };
    enum OutputId { LEFT_OUTPUT, RIGHT_OUTPUT, OUTPUTS_LEN };
    enum LightId  { LIGHTS_LEN };

    DynamicsMessage rightMessages[2];     // double buffer (if sending back)

    // Upsampler/Decimator pair.
    // Template args: <FACTOR, QUALITY>.
    dsp::Upsampler_128d<2, 8>  upSample;
    dsp::Decimator_128d<2, 8>  downSample;
    dsp::Decimator_128d<2, 8>  bandDecimators[3];

    // ── DSP state ───────────────────────────────────────────
    // LOW / MID chain, HIGH chain
    dsp::LR4_128d lpf1, hpf1; // split at FREQ1 → LOW band  &  remainder
    dsp::LR4_128d lpf2, hpf2; // split at FREQ2 → MID band  &  HIGH band

    // Per band [b]
    Band bands[3];

    // hard clip values (V)
    __m128d clipLo = _mm_set1_pd(-10.0);
    __m128d clipHi = _mm_set1_pd( 10.0);

    // parameter values
    float p_clip   = 10.f;
    float p_mix    = 1.f;
    float p_in     = 0.f;
    float p_out    = 0.f;
    float p_up     = 0.f;
    float p_down   = 0.f;
    float p_Time   = -2.0f;
    float p_freq1  = 88.3f;
    float p_freq2  = 2500.f;

    // CV mod values
    float m_mix  = 0.f;
    float m_in   = 0.f;
    float m_out  = 0.f;
    float m_up   = 0.f;
    float m_down = 0.f;

    // other internal values
    float c_up = 0.f;
    float c_down = 0.f;
    float c_bandBaseDB[3] = {0.f};

    float lastSampleRate = 0.f;
    float sr = 0.f;
    int oversampleMode = 0;
    int os = 0;

    bool updateParams = true;
    bool updateFilters = true;
    bool updateEnvelopes = true;
    bool fastFpEnabled = false;

    // ── Constructor ─────────────────────────────────────────
    Dynamics() {

        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(IN_PARAM,  -24.f, 24.f, 0.f, "Input Gain",  " dB");
        configParam(OUT_PARAM, -24.f, 24.f, 0.f, "Output Gain", " dB");
        configParam<ui::TimeParamQuantity>(TIME_PARAM,  -1.f, 1.f, 0.f, "Time");
        configParam<ui::PercentParamQuantity>(MIX_PARAM, 0.f, 1.f, 1.f, "Mix");
        configParam<ui::PercentParamQuantity>(UP_PARAM, 0.f, 2.f, 1.0f, "Upward");
        configParam<ui::PercentParamQuantity>(DOWN_PARAM, 0.f, 2.f, 1.0f, "Downward");

        const std::string bNames[3] = {"High", "Mid", "Low"};
        const float DB_MID[3]   = {-32.11f, -29.96f, -31.26f};

        for (int b = 0; b < 3; ++b) {
            configParam(GAIN_PARAM_0 + b, -24.f, 24.f, 0.f, bNames[b] + " Gain", " dB");
            configParam(THRESH_PARAM_0 + b, DB_MIN, DB_MAX, DB_MID[b], bNames[b] + " Thresh", "dB");
            configParam(DOWN_ACTIVE_PARAM_0 + b, 0.f, 1.f, 1.f, bNames[b] + " Down Active");
            configParam(UP_ACTIVE_PARAM_0 + b, 0.f, 1.f, 1.f, bNames[b] + " Up Active");
        }

        configParam(LOW_CUTOFF_PARAM, 30.f, 3000.f, 88.3f, "Low Point", " Hz");
        configParam(HIGH_CUTOFF_PARAM, 300.f, 15000.f, 2500.f, "High Point", " Hz");
        configParam(HARD_CLIP_PARAM, 5.f, 15.f, 10.f, "Hard Clip Output", " V");

        configInput(LEFT_INPUT,   "Left / Mono");
        configInput(RIGHT_INPUT,   "Right");
        configOutput(LEFT_OUTPUT, "Left / Mono");
        configOutput(RIGHT_OUTPUT, "Right");

        // exempt these params from randomization
        getParamQuantity(OUT_PARAM)->randomizeEnabled = false;
        getParamQuantity(LOW_CUTOFF_PARAM)->randomizeEnabled = false;
        getParamQuantity(HIGH_CUTOFF_PARAM)->randomizeEnabled = false;
        getParamQuantity(HARD_CLIP_PARAM)->randomizeEnabled = false;

        for (int b = 0; b < 3; ++b) {
          getParamQuantity(GAIN_PARAM_0 + b)->randomizeEnabled = false;
          getParamQuantity(DOWN_ACTIVE_PARAM_0 + b)->randomizeEnabled = false;
          getParamQuantity(UP_ACTIVE_PARAM_0 + b)->randomizeEnabled = false;
        }

        rightMessages[0] = DynamicsMessage{};
        rightMessages[1] = DynamicsMessage{};

        // Tell Rack this module accepts an expander on the right
        rightExpander.producerMessage = rightMessages;
        rightExpander.consumerMessage = rightMessages + 1;

        // not a polyphonic plugin
        outputs[LEFT_OUTPUT].setChannels(1);
        outputs[RIGHT_OUTPUT].setChannels(1);
    }

    // ── Helpers ─────────────────────────────────────────────

    void onSampleRateChange (const SampleRateChangeEvent &e) override {

        lastSampleRate = e.sampleRate;
        sr = lastSampleRate * (1 << os);
        updateFilters = updateParams = updateEnvelopes = true;
    }

    void updateOversampleMode() {

        onReset();

        os = clamp(oversampleMode, 0, 1);
        sr = lastSampleRate * (1 << os);

        updateFilters = true;
        updateEnvelopes = true;
    }

    void rebuildFilters() {

        lpf1.setFreq(p_freq1, sr, true);
        hpf1.setFreq(p_freq1, sr, false);
        lpf2.setFreq(p_freq2, sr, true);
        hpf2.setFreq(p_freq2, sr, false);
        updateFilters = false;
    }

    void rebuildEnvelopes() {

        constexpr float ATTACK_MS[3]  = {5.62914f, 9.3443724f, 11.9900682f};
        constexpr float RELEASE_MS[3] = {10.353875f, 21.784553f, 21.784553f};

        const float ts = fastpow10f(clamp(p_Time, -.3f, 1.f));
        const float tc  = ts * 0.001f * sr;

        for (int b = 0; b < 3; ++b) {
            bands[b].env.ma = fastexpf(-1.f / (ATTACK_MS[b] * tc));
            bands[b].env.mr = fastexpf(-1.f / (RELEASE_MS[b] * tc));
        }

        updateEnvelopes = false;
    }

    void onReset() override {

        lpf1.reset();
        hpf1.reset();
        lpf2.reset();
        hpf2.reset();
        upSample.reset();
        downSample.reset();

        for (int b = 0; b < 3; ++b) {
            bands[b].env.reset();
            bandDecimators[b].reset();
        }

        updateParams = true;
    }

    void updateParameters() {

        // OTT-Style constants
        constexpr float UP_COEF = 0.760191846523f; // 1 - 1/4.17
        constexpr float DOWN_COEF = -0.985007496252f; // -(1 - 1/66.7)
        constexpr float IN_GAIN_DB_OS = 5.2;
        constexpr float OUT_GAIN_DB_OS = -1.5;
        constexpr float B_GAIN_DB_OS[3] = {10.3f, 5.7f, 10.3f};

        updateParams = false;

        p_mix = params[MIX_PARAM].getValue();
        p_out = params[OUT_PARAM].getValue() + OUT_GAIN_DB_OS;
        p_in = params[IN_PARAM].getValue() + IN_GAIN_DB_OS;
        p_down = params[DOWN_PARAM].getValue();
        p_up = params[UP_PARAM].getValue();
        p_clip = params[HARD_CLIP_PARAM].getValue();

        clipLo = _mm_set1_pd(-p_clip);
        clipHi = _mm_set1_pd( p_clip);
        c_up = p_up * UP_COEF;
        c_down = p_down * DOWN_COEF;
        const float staticDB = p_in + p_out;

        float p;

        for (int b = 0; b < 3; ++b) {

            bands[b].gaindB = params[GAIN_PARAM_0 + b].getValue() + B_GAIN_DB_OS[b];

            p = params[THRESH_PARAM_0 + b].getValue();
            bands[b].downdB = p + DB_DELTA[b];
            bands[b].updB   = p - DB_DELTA[b];
            bands[b].downOn = params[DOWN_ACTIVE_PARAM_0 + b].getValue();
            bands[b].upOn   = params[UP_ACTIVE_PARAM_0 + b].getValue();
            c_bandBaseDB[b] = p_mix * (staticDB + bands[b].gaindB);
        }

        p = params[TIME_PARAM].getValue();
        if (p_Time != p) {
            p_Time = p;
            updateEnvelopes = true;
        }

        p = params[LOW_CUTOFF_PARAM].getValue();
        if (p_freq1 != p) {
            p_freq1 = p;
            updateFilters = true;
        }

        p = params[HIGH_CUTOFF_PARAM].getValue();
        if (p_freq2 != p) {
            p_freq2 = p;
            updateFilters = true;
        }

        if (os != oversampleMode)
            updateOversampleMode();

        if (updateFilters)
            rebuildFilters();

        if (updateEnvelopes)
            rebuildEnvelopes();
    }

    // ── Process ─────────────────────────────────────────────

    void 	processBypass (const ProcessArgs &args) override {

         updateParams = true;

         // needed so that the LCD display updates when no audio in or bypassed
         const int64_t frame = args.frame + getId();
         if ((frame & BYPASS_INTERVAL) == 0)
         {
             float p;
             for (int b = 0; b < 3; ++b) {
                 p = params[THRESH_PARAM_0 + b].getValue();
                 bands[b].downdB = p + DB_DELTA[b];
                 bands[b].updB   = p - DB_DELTA[b];
                 bands[b].downOn = params[DOWN_ACTIVE_PARAM_0 + b].getValue();
                 bands[b].upOn   = params[UP_ACTIVE_PARAM_0 + b].getValue();
             }
         }

         const float inL = inputs[LEFT_INPUT].getVoltageSum();
         const float inR = inputs[RIGHT_INPUT].isConnected()
             ? inputs[RIGHT_INPUT].getVoltageSum()
             : inL;

         outputs[LEFT_OUTPUT].setVoltage(inL);
         outputs[RIGHT_OUTPUT].setVoltage(inR);
    }

    void process(const ProcessArgs& args) override {

          // don't run if not connected
          if (!inputs[LEFT_INPUT].isConnected()) {
              processBypass(args);
              return;
          }

          if (!fastFpEnabled) {
              enable_fast_fp();
              fastFpEnabled = true;
          }

          // update parameters every interval
          const int64_t frame = args.frame + getId();
          if (updateParams || ((frame & PARAM_INTERVAL) == 0))
              updateParameters();

          // ── Stereo input ─────────────────────────────────────
          const bool isStereo = inputs[RIGHT_INPUT].isConnected();
          const float inL = inputs[LEFT_INPUT].getVoltageSum();
          const float inR = isStereo ? inputs[RIGHT_INPUT].getVoltageSum() : inL;

          // process left and right stereo channels at the same time with SSE
          __m128d s_input = _mm_set_pd(inR, inL);

          // Read expander CV before DSP, then send band signals after DSP.
          const bool expanderPresent = (rightExpander.module &&
                                        rightExpander.module->model == modelDynamicsExpander &&
                                        !rightExpander.module->isBypassed());

          if (expanderPresent) {

              DynamicsExpanderMessage* msg =
                (DynamicsExpanderMessage*) rightExpander.module->leftExpander.consumerMessage;

              // Read CV mod values
              m_mix  = msg->mix;
              m_up   = msg->up;
              m_down = msg->down;
              m_in   = msg->in;
              m_out  = msg->out;

              if (os == 0) {
                  // No Oversampling
                  s_input = processStereoMod(s_input);
              }
              else {
                  // 2x Oversampling
                  __m128d bandOS[3][2];
                  __m128d buf[2];

                  upSample.process(s_input, buf);

                  buf[0] = processStereoMod(buf[0]);
                  for (int b = 0; b < 3; ++b)
                      bandOS[b][0] = bands[b].output;

                  buf[1] = processStereoMod(buf[1]);
                  for (int b = 0; b < 3; ++b)
                      bandOS[b][1] = bands[b].output;

                  s_input = downSample.process(buf);

                  for (int b = 0; b < 3; ++b)
                      bands[b].output = bandDecimators[b].process(bandOS[b]);
              }

              DynamicsMessage* toSend =
                (DynamicsMessage*) rightExpander.producerMessage;

              toSend->clip = p_clip;

              for (int b = 0; b < 3; ++b) {
                  _mm_store_pd(toSend->bands[b], bands[b].output);
                  toSend->envs[b] = bands[b].outdB;
              }

              rightExpander.messageFlipRequested = true;
          }
          else {

            m_mix = m_up = m_down = m_in = m_out = 0.f;

            if (os == 0) {
                // No Oversampling
                s_input = processStereo(s_input);
            }
            else {
                // 2x Oversampling
                __m128d buf[2];
                upSample.process(s_input, buf);
                buf[0] = processStereo(buf[0]);
                buf[1] = processStereo(buf[1]);
                s_input = downSample.process(buf);
            }
          }

          alignas(16) double s_output[2];
          _mm_store_pd(s_output, s_input);

          // write to outputs
          outputs[LEFT_OUTPUT].setVoltage((float) s_output[0]);
          outputs[RIGHT_OUTPUT].setVoltage((float) s_output[1]);

    }

    template <int B>
    inline __m128d processBand(__m128d bandIn, float inDB, float mix, float down,
                               float up, float baseDB) {

        constexpr float NORM_V2 = 0.04f; // (1/5V)^2

        // take the maximum squared value of stereo channels to compute envelope
        // stereo-linked compression preserves stereo image of the audio.
        const double detect = max_f64_pd(_mm_mul_pd(bandIn, bandIn));
        const float sig2 = static_cast<float>(detect);

        // Normalised level (NORM_V = 5 V → 0 dBFS)
        const float env2 = bands[B].env.process(sig2) * NORM_V2;

        // Convert to dB. Guard against log(0): floor at –120 dBFS
        const float envDB = (env2 < 1e-12f) ? -120.f : 10.f * fastlog10f(env2) + inDB;

        // Downward compression (tame loud signals)
        float d = envDB - bands[B].downdB;
        float gainDB = d > 0.f ? bands[B].downOn * d * down : 0.f;

        // Upward compression (lift quiet signals)
        d = bands[B].updB - envDB;
        gainDB += d > 0.f ? bands[B].upOn * d * up : 0.f;

        // Safety clamp (prevent extreme gain swings)
        gainDB = mix * clamp(gainDB, -60.f, 36.f);

        // VU display meters
        bands[B].indB  = envDB;
        bands[B].outdB = envDB + gainDB;

        // scale bands by gains and add it to the output
        const float bandGain = dB2gainf(baseDB + gainDB);
        bands[B].output = _mm_mul_pd(bandIn, _mm_set1_pd(bandGain));

        return bands[B].output;
    }

    inline __m128d processStereoCore(__m128d input, float inDB, float mix, float up,
                                  float down, const float* bandBaseDB) {

        // Per-sample processing of a stereo audio signal

        // ── 3-band LR4 crossover ──────────────────────────────
        const __m128d low  = lpf1.process(input);
        const __m128d hp1  = hpf1.process(input);
        const __m128d mid  = lpf2.process(hp1);
        const __m128d high = hpf2.process(hp1);

        __m128d output = _mm_setzero_pd();

        output = _mm_add_pd(output, processBand<0>(high, inDB, mix, down, up, bandBaseDB[0]));
        output = _mm_add_pd(output, processBand<1>(mid,  inDB, mix, down, up, bandBaseDB[1]));
        output = _mm_add_pd(output, processBand<2>(low,  inDB, mix, down, up, bandBaseDB[2]));

        // safety clip signal +/- p_clip Volts
        return _mm_min_pd(_mm_max_pd(output, clipLo), clipHi);
    }

    inline __m128d processStereo(__m128d input) {
        return processStereoCore(input, p_in, p_mix, c_up, c_down, c_bandBaseDB);
    }

    inline __m128d processStereoMod(__m128d input) {

        constexpr float UP_COEF = 0.760191846523f; // 1 - 1/4.17
        constexpr float DOWN_COEF = -0.985007496252f; // -(1 - 1/66.7)

        const float inDB     = clamp(p_in + m_in, -48.f, 48.f);
        const float outDB    = clamp(p_out + m_out, -48.f, 48.f);
        const float mix      = clamp(p_mix + m_mix, 0.f, 1.f);
        const float up       = clamp(p_up + m_up, 0.f, 2.f) * UP_COEF;
        const float down     = clamp(p_down + m_down, 0.f, 2.f) * DOWN_COEF;
        const float staticDB = inDB + outDB;

        float bandBaseDB[3] = {
            mix * (staticDB + bands[0].gaindB),
            mix * (staticDB + bands[1].gaindB),
            mix * (staticDB + bands[2].gaindB)
        };

        return processStereoCore(input, inDB, mix, up, down, bandBaseDB);
    }

    // Returns true if the effective theme is dark
    bool isDarkTheme(PanelTheme theme) {
        if (theme == PanelTheme::DARK) return true;
        if (theme == PanelTheme::LIGHT) return false;
        // AUTO: follow Rack's global preference
        return settings::preferDarkPanels;
    }

    // --- Serialize the setting ---
    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "panelTheme", json_integer((int)panelTheme));
        json_object_set_new(rootJ, "oversampleMode", json_integer(oversampleMode));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* modeJ = json_object_get(rootJ, "oversampleMode");
        if (modeJ)
            oversampleMode = clamp((int) json_integer_value(modeJ), 0, 1);

        json_t* themeJ = json_object_get(rootJ, "panelTheme");
        if (themeJ) {
            int t = clamp((int) json_integer_value(themeJ), 0, 2);
            panelTheme = (PanelTheme) t;
        }

        updateParams = true;
    }
};


// ──────────────────────────────────────────────
// VU-meter display widget
// ──────────────────────────────────────────────
struct VuMeterDisplay : TransparentWidget {

    Dynamics* module = nullptr;

    VuMeterDisplay() {

    }

    // ── Map dB value → normalised height  (0 = bottom, 1 = top) ──
    static inline float dbToNorm(float db) {
        return clamp(1.f - (db - DB_MIN) / (DB_MAX - DB_MIN), 0.f, 1.f);   // linear in dB
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) return;

        static const NVGcolor LCD_BLACK   = nvgRGB(0x07, 0x08, 0x0A);
        static const NVGcolor LCD_WHITE   = nvgRGB(0xff, 0xff, 0xff);

        constexpr float INIT_DOWN_DB[3] = {-29.46f, -24.16f, -27.76f};
        constexpr float INIT_UP_DB[3] = {-34.76f, -35.76f, -34.76f};

        // display dimensions
        const float W = box.size.x;
        const float H = box.size.y;

        // bar dimensions
        const float Bar_W = 0.294550155537f * W;
        const float Bar_G = 0.203812920038f * Bar_W;
        const float Bar_WG = Bar_W + Bar_G;

        // uv line dimensions
        const float uvBW = 0.75f * Bar_W;
        const float uvLW = Bar_G * 0.8f;
        const float uvTK = 0.75f * uvLW;
        const float uvOS = (Bar_W - uvLW) * 0.5f;
        const float uvOS2 = (Bar_W - uvBW) * 0.5f;

        float x = 0.f;
        float v1, v2, v3, v4, a1, a2;

        nvgSave(args.vg);
        nvgScissor(args.vg, 0, 0, W, H);
        // ── Draw the three meters ──
        for (int i = 0; i < 3; i++) {

            const int j = 2 - i;

            if (module) {
              v1 = module->bands[j].downdB;
              v2 = module->bands[j].updB;
              v3 = module->bands[j].outdB;
              v4 = module->bands[j].indB;
              a1 = module->bands[j].downOn;
              a2 = module->bands[j].upOn;
            }
            else {
                v1 = INIT_DOWN_DB[j];
                v2 = INIT_UP_DB[j];
                v3 = DB_MIN;
                v4 = DB_MIN;
                a1 = 0.f;
                a2 = 0.f;
            }

            float down = H * dbToNorm(v1);
            float up   = H * dbToNorm(v2);
            float sigO = H * dbToNorm(v3);
            float sigI = H * dbToNorm(v4);

            // fill handle
            nvgBeginPath(args.vg);
            nvgRect(args.vg, x - 1.f, down, Bar_W + 2.f, up - down);
            nvgFillColor(args.vg, LCD_BLACK);
            nvgFill(args.vg);

            if (module) {

                // fill deactivated parts with black
                nvgBeginPath(args.vg);
                nvgRect(args.vg, x - 1.f, 0.f, Bar_W + 2.f, down + 1.f);
                nvgFillColor(args.vg, color::alpha(LCD_BLACK, 1.f - a1));
                nvgFill(args.vg);

                nvgBeginPath(args.vg);
                nvgRect(args.vg, x - 1.f, up - 1.f, Bar_W + 2.f, H - up + 1.f);
                nvgFillColor(args.vg, color::alpha(LCD_BLACK, 1.f - a2));
                nvgFill(args.vg);

                // Filled level bars
                nvgBeginPath(args.vg);
                nvgRect(args.vg, x + uvOS, sigO, uvLW, H - sigO);
                nvgFillColor(args.vg, color::alpha(LCD_WHITE, 0.75f));
                nvgFill(args.vg);

                nvgBeginPath(args.vg);
                nvgRect(args.vg, x + uvOS2, sigI - 0.5f * uvTK, uvBW, uvTK);
                nvgFillColor(args.vg, LCD_WHITE);
                nvgFill(args.vg);
            }

            // increment position
            x += Bar_WG;
        }

        nvgRestore(args.vg);

        TransparentWidget::drawLayer(args, layer);
    }
};

// Threshold sliders. Invisible, but interactive. Placed on top of the LCD.
struct InvisibleSlider : SliderKnob {

    Vec minHandlePos;
    Vec maxHandlePos;
    int aboveParamId = -1;  // param to toggle on right-click above
    int belowParamId = -1;  // param to toggle on right-click below

    InvisibleSlider(Vec size) {
        box.size = size;
        minHandlePos = Vec(0.f, size.y - 1.f);  // bottom → min
        maxHandlePos = Vec(0.f, 0.f);            // top    → max
    }

    void onHover(const HoverEvent& e) override {
        Widget::onHover(e);  // skip ParamWidget::onHover — that's where the tooltip is created
    }

    void onLeave(const LeaveEvent& e) override {
        Widget::onLeave(e);  // match the same level
    }

    void onButton(const ButtonEvent& e) override {
        if (e.button == GLFW_MOUSE_BUTTON_RIGHT && e.action == GLFW_PRESS) {
            e.consume(this);

            if (getParamQuantity()) {
                float t = getParamQuantity()->getScaledValue();
                float handleY = math::crossfade(minHandlePos.y, maxHandlePos.y, t);

                // Which param to toggle?
                int targetId = (e.pos.y < handleY) ? aboveParamId : belowParamId;

                if (targetId >= 0 && module) {
                    // Toggle the boolean param
                    float current = module->params[targetId].getValue();
                    module->params[targetId].setValue(current > 0.5f ? 0.f : 1.f);
                }
            }
            return;
        }

        SliderKnob::onButton(e);
    }

    void draw(const DrawArgs& args) override {

    }
};


// ─────────────────────────────────────────────────────────────
//    Widget  (panel + component placement)
// ─────────────────────────────────────────────────────────────
struct DynamicsWidget : ModuleWidget {

    SvgPanel* svgPanel = nullptr;  // store it directly
    bool lastDark = false;
    bool initialized = false;

    DynamicsWidget(Dynamics* module) {
        setModule(module);
        svgPanel = createPanel(asset::plugin(pluginInstance, "res/Dynamics.svg"));
        setPanel(svgPanel);


        // ── VU-meter LCD display ───────────────
        VuMeterDisplay* display = createWidget<VuMeterDisplay>(mm2px(Vec(2.909, 41.433)));
        display->box.size = mm2px(Vec(24.845, 31.505));
        display->module   = module;   // may be nullptr in the browser – draw() handles that
        addChild(display);

        auto* slider0 = new InvisibleSlider(mm2px(Vec(7.291, 31.505)));
        slider0->box.pos = mm2px(Vec(2.909, 41.433));  // position on panel
        slider0->module = module;
        slider0->paramId = Dynamics::THRESH_PARAM_2;
        slider0->aboveParamId = Dynamics::DOWN_ACTIVE_PARAM_2;
        slider0->belowParamId = Dynamics::UP_ACTIVE_PARAM_2;
        slider0->initParamQuantity();
        addParam(slider0);

        auto* slider1 = new InvisibleSlider(mm2px(Vec(7.291, 31.505)));
        slider1->box.pos = mm2px(Vec(11.686, 41.433));  // position on panel
        slider1->module = module;
        slider1->paramId = Dynamics::THRESH_PARAM_1;
        slider1->aboveParamId = Dynamics::DOWN_ACTIVE_PARAM_1;
        slider1->belowParamId = Dynamics::UP_ACTIVE_PARAM_1;
        slider1->initParamQuantity();
        addParam(slider1);

        auto* slider2 = new InvisibleSlider(mm2px(Vec(7.291, 31.505)));
        slider2->box.pos = mm2px(Vec(20.372, 41.433));  // position on panel
        slider2->module = module;
        slider2->paramId = Dynamics::THRESH_PARAM_0;
        slider2->aboveParamId = Dynamics::DOWN_ACTIVE_PARAM_0;
        slider2->belowParamId = Dynamics::UP_ACTIVE_PARAM_0;
        slider2->initParamQuantity();
        addParam(slider2);


        // Screws
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(5.415, 0.328))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(20.650, 0.328))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(5.415, 123.778))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(20.650, 123.778))));

        // ── Master controls ───────────────────────
        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(7.54, 97.25)), module, Dynamics::IN_PARAM));
        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(22.67, 97.25)), module, Dynamics::OUT_PARAM));

        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(22.67, 82.43)), module, Dynamics::DOWN_PARAM));
        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(7.54, 82.43)), module, Dynamics::UP_PARAM));

        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(22.67, 22.65)), module, Dynamics::TIME_PARAM));
        addParam(createParamCentered<ui::OmegaLargeKnob>(
            mm2px(Vec(7.54,  22.65)), module, Dynamics::MIX_PARAM));

        // ── Band output gains ────────────────────────────
        addParam(createParamCentered<ui::OmegaSmallKnob>(
            mm2px(Vec(23.88, 36.86)), module, Dynamics::GAIN_PARAM_0));
        addParam(createParamCentered<ui::OmegaSmallKnob>(
            mm2px(Vec(15.10, 36.86)), module, Dynamics::GAIN_PARAM_1));
        addParam(createParamCentered<ui::OmegaSmallKnob>(
            mm2px(Vec(6.42, 36.86)), module, Dynamics::GAIN_PARAM_2));

        // ── I/O ports ───────────────────────────
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 110.5)),  module, Dynamics::LEFT_INPUT));
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 119.31)), module, Dynamics::RIGHT_INPUT));

        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(22.82, 110.5)), module, Dynamics::LEFT_OUTPUT));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(22.82, 119.31)), module, Dynamics::RIGHT_OUTPUT));
    }

    // Returns true if the effective theme is dark
    bool isDarkTheme(PanelTheme theme) {
        if (theme == PanelTheme::DARK) return true;
        if (theme == PanelTheme::LIGHT) return false;
        // AUTO: follow Rack's global preference
        return settings::preferDarkPanels;
    }

    void appendContextMenu(Menu* menu) override {
        Dynamics* m = dynamic_cast<Dynamics*>(this->module);
        if (!m) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Crossover Points"));

        auto* slider1 = new rack::ui::Slider;
        slider1->quantity = m->paramQuantities[Dynamics::LOW_CUTOFF_PARAM];
        slider1->box.size.x = 200.f; // width of the slider in the menu
        menu->addChild(slider1);

        auto* slider2 = new rack::ui::Slider;
        slider2->quantity = m->paramQuantities[Dynamics::HIGH_CUTOFF_PARAM];
        slider2->box.size.x = 200.f; // width of the slider in the menu
        menu->addChild(slider2);

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Options"));

        auto* slider3 = new rack::ui::Slider;
        slider3->quantity = m->paramQuantities[Dynamics::HARD_CLIP_PARAM];
        slider3->box.size.x = 200.f; // width of the slider in the menu
        menu->addChild(slider3);

        menu->addChild(createIndexPtrSubmenuItem(
            "Quality",
            {"Eco", "High"},
            &m->oversampleMode
        ));

        menu->addChild(createSubmenuItem("Theme", "",
            [=](Menu* subMenu) {
                auto addThemeItem = [&](const std::string& label, PanelTheme theme) {
                    subMenu->addChild(createCheckMenuItem(label, "",
                        [=]() { return m->panelTheme == theme; },
                        [=]() { m->panelTheme = theme; }
                    ));
                };

                addThemeItem("Automatic", PanelTheme::AUTO);
                addThemeItem("Light",     PanelTheme::LIGHT);
                addThemeItem("Dark",      PanelTheme::DARK);
            }
        ));

    }

    // Called every frame — only do work when the theme actually changes
    void step() override {

        bool dark;

        if (module) {
            // Normal use — respect the per-module theme choice
            Dynamics* m = dynamic_cast<Dynamics*>(module);
            dark = isDarkTheme(m->panelTheme);
        } else {
            // Browser preview — no module, just follow the global setting
            dark = settings::preferDarkPanels;
        }

        if (!initialized || dark != lastDark) {
            initialized = true;
            lastDark = dark;

            if (svgPanel) {
                svgPanel->setBackground(APP->window->loadSvg(
                    asset::plugin(pluginInstance,
                        dark ? "res/Dynamics-dark.svg"
                             : "res/Dynamics.svg")));
            }

            for (Widget* w : children) {
                ui::ThemedWidget* tw = dynamic_cast<ui::ThemedWidget*>(w);
                if (tw) tw->loadTheme(dark);
            }
        }

        ModuleWidget::step();
    }

};

Model* modelDynamics = createModel<Dynamics, DynamicsWidget>("OmegaDynamics");

} // namespace Omega
