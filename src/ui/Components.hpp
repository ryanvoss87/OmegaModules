#pragma once
#include "rack.hpp"


namespace Omega::ui {

constexpr const float KNOB_MIN_ANGLE = -2.38761042f;
constexpr const float KNOB_MAX_ANGLE =  2.38761042f;
constexpr const float SHADOW_OPACITY =  0.075f;

struct ThemedWidget {

    bool lastDark = false;
    virtual ~ThemedWidget() = default;
    virtual void loadTheme(bool dark) = 0;
};

struct OmegaSmallKnob : RoundKnob, ThemedWidget {

    OmegaSmallKnob() {
        minAngle = KNOB_MIN_ANGLE;
        maxAngle = KNOB_MAX_ANGLE;
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
      setSvg(Svg::load(asset::plugin(pluginInstance,
          dark ? "res/OmegaSmallKnob-dark.svg"
               : "res/OmegaSmallKnob.svg")));
        bg->setSvg(Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaSmallKnob_bg-dark.svg"
                 : "res/OmegaSmallKnob_bg.svg")));
        fb->dirty = true;
        lastDark = dark;
    }
};

struct OmegaMediumKnob : RoundKnob, ThemedWidget {

    OmegaMediumKnob() {
        minAngle = KNOB_MIN_ANGLE;
        maxAngle = KNOB_MAX_ANGLE;
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
      setSvg(Svg::load(asset::plugin(pluginInstance,
          dark ? "res/OmegaMediumKnob-dark.svg"
               : "res/OmegaMediumKnob.svg")));
        bg->setSvg(Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaMediumKnob_bg-dark.svg"
                 : "res/OmegaMediumKnob_bg.svg")));
        fb->dirty = true;
        lastDark = dark;
    }
};

struct OmegaLargeKnob : RoundKnob, ThemedWidget {

    OmegaLargeKnob() {
        minAngle = KNOB_MIN_ANGLE;
        maxAngle = KNOB_MAX_ANGLE;
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        setSvg(Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaLargeKnob-dark.svg"
                 : "res/OmegaLargeKnob.svg")));
        bg->setSvg(Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaLargeKnob_bg-dark.svg"
                 : "res/OmegaLargeKnob_bg.svg")));
        fb->dirty = true;
        lastDark = dark;
    }

};

struct OmegaPJ301MPort : app::SvgPort, ThemedWidget  {

    OmegaPJ301MPort() {
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        auto svg = Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaPJ301MPort-dark.svg"
                 : "res/OmegaPJ301MPort.svg"));

        if (!svg) {
            return;
        }

        setSvg(svg);
        fb->dirty = true;
        lastDark = dark;
    }
};

struct OmegaBluePJ301MPort : app::SvgPort, ThemedWidget {

    OmegaBluePJ301MPort() {
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        auto svg = Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaBluePJ301MPort-dark.svg"
                 : "res/OmegaBluePJ301MPort.svg"));

        if (!svg) {
            return;
        }

        setSvg(svg);
        fb->dirty = true;
        lastDark = dark;
    }
};

struct OmegaPinkPJ301MPort : app::SvgPort, ThemedWidget {

    OmegaPinkPJ301MPort() {
        shadow->opacity = SHADOW_OPACITY;
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        auto svg = Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaPinkPJ301MPort-dark.svg"
                 : "res/OmegaPinkPJ301MPort.svg"));

        if (!svg) {
            return;
        }

        setSvg(svg);
        fb->dirty = true;
        lastDark = dark;
    }
};

struct OmegaPurpleLight : GrayModuleLightWidget, ThemedWidget {

    OmegaPurpleLight() {
        addBaseColor(nvgRGB(0x4C, 0x3A, 0xFB)); // start with dark (purple)
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        baseColors.clear();
        if (dark)
            addBaseColor(nvgRGB(0x4C, 0x3A, 0xFB)); // purple
        else
            addBaseColor(nvgRGB(0xE9, 0xA1, 0xF4)); // pink
        lastDark = dark;
    }
};


struct OmegaThemedScrew : app::SvgScrew, ThemedWidget {

    OmegaThemedScrew() {
        loadTheme(settings::preferDarkPanels);
    }

    void loadTheme(bool dark) override {
        auto svg = Svg::load(asset::plugin(pluginInstance,
            dark ? "res/OmegaThemedScrew-dark.svg"
                 : "res/OmegaThemedScrew.svg"));

        if (!svg) {
            return;
        }

        setSvg(svg);
        fb->dirty = true;
        lastDark = dark;
    }
};

} // namespace Omega::ui
