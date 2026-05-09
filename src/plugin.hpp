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

#pragma once
#include <rack.hpp>

using namespace rack;

// Forward-declared by each module's .cpp file
extern Plugin* pluginInstance;

namespace Omega {

// Message from expander -> host
struct DynamicsExpanderMessage {
    float mix = 0.f;
    float up = 0.f;
    float down = 0.f;
    float in = 0.f;
    float out = 0.f;
};

// Message from host -> expander
struct alignas(16) DynamicsMessage {
    alignas(16) double bands[3][2] = {0.};
    float envs[3] = {0.f};
    float outputValue = 0.f;
    float clip = 10.f;
};

enum class PanelTheme {
    AUTO,   // Follow Rack's global setting
    LIGHT,
    DARK
};

extern Model* modelDynamics;
extern Model* modelDynamicsExpander;

} // namespace Omega
