#pragma once
#include "rack.hpp"

namespace Omega::ui {

struct PercentParamQuantity : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        float x = 100.f * getValue(); // 0 → 1

        int percent = (int) std::round(x);
        return rack::string::f("%d", percent);
    }

    std::string getUnit() override {
        return "%";
    }
};

struct TimeParamQuantity : rack::engine::ParamQuantity {

    std::string getDisplayValueString() override {
        float x = 100.f * std::pow(10.f, getValue());
        int percent = (int) std::round(x);
        return rack::string::f("%d", percent);
    }

    std::string getUnit() override {
        return "%";
    }
};




} // namespace Omega::ui
