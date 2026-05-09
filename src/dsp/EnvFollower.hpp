#pragma once
#include <cmath> // sqrt, abs


namespace Omega::dsp {

// ─────────────────────────────────────────────────────────────
//  Envelope Follower
// ─────────────────────────────────────────────────────────────
struct EnvFollower {

    float level = 0.f;
    float ma = 0.f; // attack coef (exp(-1/(time * sr)))
    float mr = 0.f; // release coef (exp(-1/(time * sr)))

    inline float process(float in) {

        const float m = in > level ? ma : mr;
        level = m * level + (1.f - m) * in;
        return level;
    }

    inline float processPeak(float in) {
        return process(std::abs(in));
    }

    inline float processRMS(float in) {
        return std::sqrt(process(in * in));
    }

    void reset() {
        level = 0.f;
    }

};

} // namespace Omega::dsp
