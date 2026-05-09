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
 * DynamicsExpander.cpp  –  Expander Module for Omega Dynamics
 * Must be connected to the right side of Dynamics in VCV Rack for it to work
 * Adds CV Mod inputs for MIX, IN, OUT, UP, DOWN Parameters
 * Adds CV outputs for L/M/H Band envelopes (0V to 10V)
 * Adds L/R audio outputs for L/M/H Bands
 */

#include "plugin.hpp"
#include "ui/ParamQuantities.hpp"
#include "ui/Components.hpp"

namespace Omega {

struct CvOnePole {

    float y = 0.f;
    float a = 0.f;

    void setCutoff(float cutoffHz, float sampleRate) {
        const float twopi = 6.28318530718f;
        cutoffHz = clamp(cutoffHz, 0.1f, 20000.f);
        a = 1.f - std::exp(-twopi * cutoffHz / sampleRate);
    }

    inline float process(float x) {
        y += a * (x - y);
        // Kill denormal/sub-audible residue and clamp 0-10V.
        if (y < 1e-20f) y = 0.f;
        if (y > 10.f) y = 10.f;

        return y;
    }

    inline void reset(float value = 0.f) {
        y = value;
    }
};

struct DynamicsExpander : Module {

    // ── Parameter / Port IDs ────────────────────────────────
    enum ParamId  { MIX_CV_AMT_PARAM,
                    UP_CV_AMT_PARAM,
                    DOWN_CV_AMT_PARAM,
                    IN_GAIN_CV_AMT_PARAM,
                    OUT_GAIN_CV_AMT_PARAM,
                    ENV_MIN_DB, ENV_MAX_DB,
                    PARAMS_LEN};
    enum InputId  { MIX_CV_INPUT,
                    UP_CV_INPUT, DOWN_CV_INPUT,
                    IN_GAIN_CV_INPUT, OUT_GAIN_CV_INPUT,
                    INPUTS_LEN
                  };
    enum OutputId { ENV_OUTPUT_0, ENV_OUTPUT_1, ENV_OUTPUT_2,
                    LEFT_OUTPUT_0, LEFT_OUTPUT_1, LEFT_OUTPUT_2,
                    RIGHT_OUTPUT_0, RIGHT_OUTPUT_1, RIGHT_OUTPUT_2,
                    OUTPUTS_LEN
                  };
    enum LightId  {CONNECTED_LIGHT,  LIGHTS_LEN };

    PanelTheme panelTheme = PanelTheme::AUTO;
    DynamicsExpanderMessage leftMessages[2];


    CvOnePole envSmooth[3];
    float lastSampleRate = 0.f;

    // internal params
    float p_mix  = 0.0f;
    float p_up   = 0.0f;
    float p_down = 0.0f;
    float p_in   = 0.0f;
    float p_out  = 0.0f;
    float p_emin = -70.f;
    float p_emax = 0.0f;

    float c_envScale = 10.f;
    bool invertEnvelopes = false;

    DynamicsExpander() {

        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<ui::PercentParamQuantity>(MIX_CV_AMT_PARAM, -2.f, 2.f, 0.0f, "Mix CV");
        configParam<ui::PercentParamQuantity>(UP_CV_AMT_PARAM, -2.f, 2.f, 0.0f, "Up CV");
        configParam<ui::PercentParamQuantity>(DOWN_CV_AMT_PARAM, -2.f, 2.f, 0.0f, "Down CV");
        configParam<ui::PercentParamQuantity>(IN_GAIN_CV_AMT_PARAM, -2.f, 2.f, 0.0f, "In Gain CV");
        configParam<ui::PercentParamQuantity>(OUT_GAIN_CV_AMT_PARAM, -2.f, 2.f, 0.0f, "Out Gain CV");

        const std::string bNames[3] = {"High", "Mid", "Low"};
        for (int b = 0; b < 3; ++b) {
            configOutput(ENV_OUTPUT_0   + b, bNames[b] + " Env");
            configOutput(LEFT_OUTPUT_0  + b, bNames[b] + " Left");
            configOutput(RIGHT_OUTPUT_0 + b, bNames[b] + " Right");
        }

        configParam(ENV_MIN_DB, -70.f, 0.f, -40.f, "Min", " dB");
        configParam(ENV_MAX_DB, -70.f, 0.f, -20.f, "Max", " dB");

        configInput(MIX_CV_INPUT,      "Mix CV");
        configInput(UP_CV_INPUT,       "Up CV");
        configInput(DOWN_CV_INPUT,     "Down CV");
        configInput(IN_GAIN_CV_INPUT,  "In Gain CV");
        configInput(OUT_GAIN_CV_INPUT, "Out Gain CV");

        configLight(CONNECTED_LIGHT, "Connected");


        // Zero-initialize both buffers so there's no garbage on the first read
        leftMessages[0] = DynamicsExpanderMessage{};
        leftMessages[1] = DynamicsExpanderMessage{};

        // This expander talks to a host on its LEFT
        leftExpander.producerMessage = leftMessages;
        leftExpander.consumerMessage = leftMessages + 1;
    }

    void onSampleRateChange(const SampleRateChangeEvent& e) override {
        lastSampleRate = e.sampleRate;

        // Good starting range: 20-80 Hz.
        // Lower values look smoother; higher values track fast transients better.
        constexpr float ENV_SMOOTH_CUTOFF_HZ = 60.f;

        for (int b = 0; b < 3; ++b) {
            envSmooth[b].setCutoff(ENV_SMOOTH_CUTOFF_HZ, lastSampleRate);
        }
    }

    void onReset() override {
        for (int b = 0; b < 3; ++b) {
            envSmooth[b].reset(0.f);
        }
    }

    void process(const ProcessArgs& args) override {

        if (0 == ((args.frame + getId()) & 255)){

            constexpr float CV_SCALE = 0.1f;
            constexpr float GAIN_CV_SCALE = CV_SCALE * 24.f;

            p_mix  = CV_SCALE * params[MIX_CV_AMT_PARAM].getValue();
            p_up   = CV_SCALE * params[UP_CV_AMT_PARAM].getValue();
            p_down = CV_SCALE * params[DOWN_CV_AMT_PARAM].getValue();
            p_in   = GAIN_CV_SCALE * params[IN_GAIN_CV_AMT_PARAM].getValue();
            p_out  = GAIN_CV_SCALE * params[OUT_GAIN_CV_AMT_PARAM].getValue();
            p_emin = params[ENV_MIN_DB].getValue();
            p_emax = params[ENV_MAX_DB].getValue();

            const float p_emax_m_5 = p_emax - 5.f;
            if (p_emin > p_emax_m_5)
                p_emin = p_emax_m_5;

            c_envScale = 10.f / (p_emax - p_emin);
        }

        const bool connectedToHost = (leftExpander.module &&
                                      leftExpander.module->model == modelDynamics &&
                                      !leftExpander.module->isBypassed());

        if (connectedToHost) {

            lights[CONNECTED_LIGHT].setBrightness(1.f);

            // WRITE to host
            DynamicsExpanderMessage* msg =
                (DynamicsExpanderMessage*) leftExpander.producerMessage;

            msg->mix  = inputs[MIX_CV_INPUT].isConnected()
                        ? inputs[MIX_CV_INPUT].getVoltage() * p_mix : 0.f;
            msg->up   = inputs[UP_CV_INPUT].isConnected()
                        ? inputs[UP_CV_INPUT].getVoltage() * p_up : 0.f;
            msg->down = inputs[DOWN_CV_INPUT].isConnected()
                        ? inputs[DOWN_CV_INPUT].getVoltage() * p_down : 0.f;
            msg->in   = inputs[IN_GAIN_CV_INPUT].isConnected()
                        ? inputs[IN_GAIN_CV_INPUT].getVoltage() * p_in : 0.f;
            msg->out  = inputs[OUT_GAIN_CV_INPUT].isConnected()
                        ? inputs[OUT_GAIN_CV_INPUT].getVoltage() * p_out : 0.f;

            // Flip so host sees it next tick
            leftExpander.messageFlipRequested = true;

            // READ what host sent back
            DynamicsMessage* fromHost =
               (DynamicsMessage*) leftExpander.module->rightExpander.consumerMessage;

            float clip = fromHost->clip;

            for (int b = 0; b < 3; ++b) {
              const float rawEnv = c_envScale * (fromHost->envs[b] - p_emin);
              const float smoothEnv = envSmooth[b].process(rawEnv);
              outputs[ENV_OUTPUT_0 + b].setVoltage(invertEnvelopes ? 10.f - smoothEnv : smoothEnv);
              outputs[LEFT_OUTPUT_0  + b].setVoltage(clamp((float) fromHost->bands[b][0], -clip, clip));
              outputs[RIGHT_OUTPUT_0 + b].setVoltage(clamp((float) fromHost->bands[b][1], -clip, clip));
            }
        }
        else {

            lights[CONNECTED_LIGHT].setBrightness(0.f);

            for (int b = 0; b < 3; ++b) {
                envSmooth[b].reset();
                outputs[ENV_OUTPUT_0 + b].setVoltage(0.0f);
                outputs[LEFT_OUTPUT_0 + b].setVoltage(0.0f);
                outputs[RIGHT_OUTPUT_0 + b].setVoltage(0.0f);
            }
        }
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
        json_object_set_new(rootJ, "invertEnvelopes", json_boolean(invertEnvelopes));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* themeJ = json_object_get(rootJ, "panelTheme");
        if (themeJ) {
            int t = clamp((int) json_integer_value(themeJ), 0, 2);
            panelTheme = (PanelTheme) t;
        }

        json_t* invertEnvelopesJ = json_object_get(rootJ, "invertEnvelopes");
        if (invertEnvelopesJ)
            invertEnvelopes = json_boolean_value(invertEnvelopesJ);
    }

};


struct DynamicsExpanderWidget : ModuleWidget {

    SvgPanel* svgPanel = nullptr;  // store it directly
    bool lastDark = false;
    bool initialized = false;

    DynamicsExpanderWidget(DynamicsExpander* module) {
        setModule(module);
        svgPanel = createPanel(asset::plugin(pluginInstance, "res/DynamicsExpander.svg"));
        setPanel(svgPanel);

        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(5.415, 0.328))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(20.650, 0.328))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(5.415, 123.778))));
        addChild(createWidget<ui::OmegaThemedScrew>(mm2px(Vec(20.650, 123.778))));

        // CV Mod Inputs
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74,  22.58)), module, DynamicsExpander::MIX_CV_INPUT));
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 84.01)), module, DynamicsExpander::UP_CV_INPUT));
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 95.48)), module, DynamicsExpander::DOWN_CV_INPUT));
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 106.62)), module, DynamicsExpander::IN_GAIN_CV_INPUT));
        addInput(createInputCentered<ui::OmegaPJ301MPort>(
            mm2px(Vec(7.74, 117.83)), module, DynamicsExpander::OUT_GAIN_CV_INPUT));

        // CV Attenuverters
        addParam(createParamCentered<ui::OmegaMediumKnob>(
            mm2px(Vec(22.43, 22.58)), module, DynamicsExpander::MIX_CV_AMT_PARAM));
        addParam(createParamCentered<ui::OmegaMediumKnob>(
            mm2px(Vec(22.43, 84.01)), module, DynamicsExpander::UP_CV_AMT_PARAM));
        addParam(createParamCentered<ui::OmegaMediumKnob>(
            mm2px(Vec(22.43, 95.48)), module, DynamicsExpander::DOWN_CV_AMT_PARAM));
        addParam(createParamCentered<ui::OmegaMediumKnob>(
            mm2px(Vec(22.43, 106.62)), module, DynamicsExpander::IN_GAIN_CV_AMT_PARAM));
        addParam(createParamCentered<ui::OmegaMediumKnob>(
            mm2px(Vec(22.43, 117.83)), module, DynamicsExpander::OUT_GAIN_CV_AMT_PARAM));

        // HIGH BAND
        addOutput(createOutputCentered<ui::OmegaPinkPJ301MPort>(
            mm2px(Vec(23.98, 43.91)),  module, DynamicsExpander::ENV_OUTPUT_0));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(23.98, 63.24)),  module, DynamicsExpander::LEFT_OUTPUT_0));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(23.98, 71.58)),  module, DynamicsExpander::RIGHT_OUTPUT_0));

        // MID BAND
        addOutput(createOutputCentered<ui::OmegaPinkPJ301MPort>(
            mm2px(Vec(15.27, 43.91)),  module, DynamicsExpander::ENV_OUTPUT_1));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(15.27, 63.24)),  module, DynamicsExpander::LEFT_OUTPUT_1));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(15.27, 71.58)),  module, DynamicsExpander::RIGHT_OUTPUT_1));

        // LOW BAND
        addOutput(createOutputCentered<ui::OmegaPinkPJ301MPort>(
            mm2px(Vec(6.50, 43.91)),  module, DynamicsExpander::ENV_OUTPUT_2));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(6.50, 63.24)),  module, DynamicsExpander::LEFT_OUTPUT_2));
        addOutput(createOutputCentered<ui::OmegaBluePJ301MPort>(
            mm2px(Vec(6.50, 71.58)),  module, DynamicsExpander::RIGHT_OUTPUT_2));

        addChild(createLightCentered<SmallLight<ui::OmegaPurpleLight>>(
            mm2px(Vec(15.23, 2.58)), module, DynamicsExpander::CONNECTED_LIGHT));
    }

    // Returns true if the effective theme is dark
    bool isDarkTheme(PanelTheme theme) {
        if (theme == PanelTheme::DARK) return true;
        if (theme == PanelTheme::LIGHT) return false;
        // AUTO: follow Rack's global preference
        return settings::preferDarkPanels;
    }

    void appendContextMenu(Menu* menu) override {
        DynamicsExpander* m = dynamic_cast<DynamicsExpander*>(this->module);
        if (!m) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Band Envs Range"));

        auto* slider1 = new rack::ui::Slider;
        slider1->quantity = m->paramQuantities[DynamicsExpander::ENV_MAX_DB];
        slider1->box.size.x = 200.f; // width of the slider in the menu
        menu->addChild(slider1);

        auto* slider2 = new rack::ui::Slider;
        slider2->quantity = m->paramQuantities[DynamicsExpander::ENV_MIN_DB];
        slider2->box.size.x = 200.f; // width of the slider in the menu
        menu->addChild(slider2);

        menu->addChild(createBoolPtrMenuItem(
            "Invert",
            "",
            &m->invertEnvelopes
        ));

        menu->addChild(new MenuSeparator);
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
            DynamicsExpander* m = dynamic_cast<DynamicsExpander*>(module);
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
                        dark ? "res/DynamicsExpander-dark.svg"
                             : "res/DynamicsExpander.svg")));
            }

            for (Widget* w : children) {
                ui::ThemedWidget* tw = dynamic_cast<ui::ThemedWidget*>(w);
                if (tw) tw->loadTheme(dark);
            }
        }

        ModuleWidget::step();
    }

};

Model* modelDynamicsExpander = createModel<DynamicsExpander, DynamicsExpanderWidget>("OmegaDynamicsExpander");

} // namespace Omega
