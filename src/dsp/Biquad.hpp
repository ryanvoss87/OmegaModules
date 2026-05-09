#pragma once
#include <cmath> // sin, cos
#include <emmintrin.h>  // SSE2: __m128d, _mm_*_pd

namespace Omega::dsp {

// ─────────────────────────────────────────────────────────────
//  Biquad filter  –  Direct Form II Transposed (numerically stable)
// ─────────────────────────────────────────────────────────────
struct Biquad {
    // Coefficients
    double b0 = 1.0, b1 = 0.0, b2 = 0.0;
    double a1 = 0.0, a2 = 0.0;
    // Delay lines
    double s1 = 0.0, s2 = 0.0;

    void setCoeffs(double _b0, double _b1, double _b2,
                   double _a1, double _a2) {
        b0 = _b0; b1 = _b1; b2 = _b2;
        a1 = _a1; a2 = _a2;
    }

    inline float process(float x) {
        double xd = static_cast<double>(x);
        double y  = b0 * xd + s1;
        s1        = b1 * xd - a1 * y + s2;
        s2        = b2 * xd - a2 * y;
        return static_cast<float>(y);
    }

    void reset() { s1 = s2 = 0.0; }
};

struct Biquad_128d {
    // Coefficients
    __m128d b0, b1, b2;
    __m128d a1, a2;
    __m128d s1, s2;

    Biquad_128d() {
        b0 = b1 = b2 = a1 = a2 = s1 = s2 = _mm_set1_pd(0.0);
    }

    void setCoeffs(double _b0, double _b1, double _b2,
                   double _a1, double _a2) {
        b0 = _mm_set1_pd(_b0); b1 = _mm_set1_pd(_b1); b2 = _mm_set1_pd(_b2);
        a1 = _mm_set1_pd(_a1); a2 = _mm_set1_pd(_a2);
    }

    inline __m128d process(__m128d xd) {
      __m128d y  = _mm_add_pd(_mm_mul_pd(b0, xd), s1);
      s1 = _mm_add_pd(_mm_sub_pd(_mm_mul_pd(b1, xd), _mm_mul_pd(a1, y)), s2);
      s2 = _mm_sub_pd(_mm_mul_pd(b2, xd), _mm_mul_pd(a2, y));
        return y;
    }

    void reset() { s1 = s2 = _mm_set1_pd(0.0); }
};

// ─────────────────────────────────────────────────────────────
//   LR4 filter  –  Linkwitz-Riley 4th order
//      = two cascaded 2nd-order Butterworth filters of the same type
//      Property: LP4(f) + HP4(f) = unity across all frequencies
// ─────────────────────────────────────────────────────────────
struct LR4 {
    Biquad bq[2]; // cascade

    void setFreq(double freq, double sampleRate, bool isLowpass) {

        const double twopi = 6.2831853071795864769;
        const double w0    = twopi * freq / sampleRate;
        const double cosW0 = std::cos(w0);
        const double sinW0 = std::sin(w0);
        const double alpha = sinW0 * 0.7071067811865475244; // Q = 1/√2  (Butterworth)

        double b0, b1, b2;
        if (isLowpass) {
            b0 = (1.0 - cosW0) * 0.5;
            b1 =  1.0 - cosW0;
            b2 = (1.0 - cosW0) * 0.5;
        } else {
            b0 = (1.0 + cosW0) * 0.5;
            b1 = -(1.0 + cosW0);
            b2 = (1.0 + cosW0) * 0.5;
        }
        double a0 =  1.0 + alpha;
        double a1 = -2.0 * cosW0;
        double a2 =  1.0 - alpha;

        // Normalise by a0
        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;

        bq[0].setCoeffs(b0, b1, b2, a1, a2);
        bq[1].setCoeffs(b0, b1, b2, a1, a2);
    }

    inline float process(float x) {
        return bq[1].process(bq[0].process(x));
    }

    void reset() { bq[0].reset(); bq[1].reset(); }
};

struct LR4_128d {
    Biquad_128d bq[2]; // cascade

    void setFreq(double freq, double sampleRate, bool isLowpass) {

        const double twopi = 6.2831853071795864769;
        const double w0    = twopi * freq / sampleRate;
        const double cosW0 = std::cos(w0);
        const double sinW0 = std::sin(w0);
        const double alpha = sinW0 * 0.7071067811865475244; // Q = 1/√2  (Butterworth)

        double b0, b1, b2;
        if (isLowpass) {
            b0 = (1.0 - cosW0) * 0.5;
            b1 =  1.0 - cosW0;
            b2 = (1.0 - cosW0) * 0.5;
        } else {
            b0 = (1.0 + cosW0) * 0.5;
            b1 = -(1.0 + cosW0);
            b2 = (1.0 + cosW0) * 0.5;
        }
        double a0 =  1.0 + alpha;
        double a1 = -2.0 * cosW0;
        double a2 =  1.0 - alpha;

        // Normalise by a0
        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;

        bq[0].setCoeffs(b0, b1, b2, a1, a2);
        bq[1].setCoeffs(b0, b1, b2, a1, a2);
    }

    inline __m128d process(__m128d x) {
        return bq[1].process(bq[0].process(x));
    }

    void reset() { bq[0].reset(); bq[1].reset(); }
};

} // namespace Omega::dsp
