#pragma once

#include <cstring> // memcpy
#include <cstdint> // uint32_t, int32_t
#include <limits> // infinity, NaN
#include <xmmintrin.h> // flush zero
#include <emmintrin.h> // __m128d, _mm_*_pd


inline void enable_fast_fp() {
#if defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || defined(_M_IX86)

    #ifdef _MM_FLUSH_ZERO_ON
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    #endif

    #ifdef _MM_DENORMALS_ZERO_ON
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    #endif

#endif
}

static_assert(std::numeric_limits<float>::is_iec559, "Requires IEEE 754 float");
static_assert(sizeof(float) == sizeof(uint32_t),     "Requires 32-bit float");

// ----------------------------------------------------------------------------
// Bit-punning helpers — defined behavior via memcpy
// ----------------------------------------------------------------------------

static inline uint32_t float_to_bits(const float f){
    uint32_t i;
    memcpy(&i, &f, sizeof(i));
    return i;
}

static inline float bits_to_float(const uint32_t i){
    float f;
    memcpy(&f, &i, sizeof(f));
    return f;
}

// ----------------------------------------------------------------------------
// fastpow2f — approximates 2^x
// Valid for x in (-126, 127); clamped outside that range
// ----------------------------------------------------------------------------

static inline float fastpow2f(const float x) noexcept {
    const float clipp  = (x < -126.0f) ? -126.0f : (x > 127.0f) ? 127.0f : x;
    const float offset = (x < 0.0f) ? 1.0f : 0.0f;
    const int   w      = static_cast<int>(clipp);
    const float z      = clipp - static_cast<float>(w) + offset;

    const uint32_t bits = static_cast<uint32_t>(
        static_cast<int32_t>(
            (1u << 23) * (clipp + 121.2740575f
                                + 27.7280233f / (4.84252568f - z)
                                - 1.49012907f * z)
        )
    );

    return bits_to_float(bits);
}

// ----------------------------------------------------------------------------
// fastlog2f — approximates log2(x)
// Returns -inf for x == 0, quiet NaN for x < 0 (matching std::log2 behavior)
// ----------------------------------------------------------------------------

static inline float fastlog2f(const float x) noexcept{
    if (x <= 0.0f)
        return (x == 0.0f) ? -std::numeric_limits<float>::infinity()
                           :  std::numeric_limits<float>::quiet_NaN();

    const uint32_t vx_i = float_to_bits(x);
    const uint32_t mx_i = (vx_i & 0x007FFFFFu) | 0x3F000000u;
    const float    mx_f = bits_to_float(mx_i);
    const float    y    = static_cast<float>(vx_i);

    // 1.1920928955078125e-7 == 1.0 / (1 << 23)
    return (1.1920928955078125e-7f * y - 124.22551499f
            - 1.498030302f  * mx_f
            - 1.72587999f   / (0.3520887068f + mx_f));
}

static inline float fastlog10f (const float x){
    // return y = Log10(x)
    return 0.30103f * fastlog2f(x);
}

static inline float fastpowf(const float x, const float p){
    // return y = x^p
    return fastpow2f(p * fastlog2f(x));
}

static inline float fastpow10f(const float x){
    // return y = 10^x
    return fastpow2f(x * 3.321928094887362348f);
}

static inline float fastexpf(const float x){
    // return y = e^x
    return fastpow2f(x * 1.44269504088896341f);
}

static inline float dB2gainf(const float dB){
    return fastpow10f(0.05f * dB);
}

static inline float gain2dBf(const float gain){
    return 20.f * fastlog10f(gain);
}

static inline __m128d swap_pd(const __m128d x){
    return _mm_shuffle_pd(x, x, _MM_SHUFFLE2(0, 1));
}

static inline double max_f64_pd(const __m128d x){
    return _mm_cvtsd_f64(_mm_max_sd(x, swap_pd(x)));
}
