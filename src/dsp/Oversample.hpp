// Modifications to Rack's dsp::Upsampler and dsp::Decimator to take __m128d type values

#pragma once

#include <cstring>      // memmove, memset, memcpy
#include <emmintrin.h>  // SSE2: __m128d, _mm_*_pd
#include "rack.hpp"


namespace Omega::dsp {

template <int OVERSAMPLE, int QUALITY>
struct Decimator_128d {

    static constexpr int N = OVERSAMPLE * QUALITY;

    alignas(16) __m128d buffer[2 * N];
    alignas(16) __m128d kernelPhase[OVERSAMPLE][QUALITY];

    int inIndex;

    Decimator_128d(double cutoff = 0.9) {
        float kernel_f[N];

        rack::dsp::boxcarLowpassIR(kernel_f, N, (float)(cutoff * 0.5 / OVERSAMPLE));
        rack::dsp::blackmanHarrisWindow(kernel_f, N);

        // Polyphase split
        for (int p = 0; p < OVERSAMPLE; p++) {
            for (int j = 0; j < QUALITY; j++) {
                int k = OVERSAMPLE * j + p;
                kernelPhase[p][j] = _mm_set1_pd((double)kernel_f[k]);
            }
        }

        reset();
    }

    void reset() {
        inIndex = 0;
        for (int i = 0; i < 2 * N; i++)
            buffer[i] = _mm_setzero_pd();
    }

    /** in: OVERSAMPLE frames */
    __m128d process(__m128d* in) {
          // write + mirror
        for (int i = 0; i < OVERSAMPLE; i++) {
            int idx = inIndex + i;
            buffer[idx] = in[i];
            buffer[idx + N] = in[i];
        }

        inIndex += OVERSAMPLE;
        if (inIndex >= N)
            inIndex -= N;

        __m128d* buf = &buffer[inIndex + N - 1];

        __m128d out = _mm_setzero_pd();

        // Sequential buf access: buf[0], buf[-1], buf[-2], ...
        for (int j = 0; j < QUALITY; j++) {
            for (int p = 0; p < OVERSAMPLE; p++) {
                int offset = p + j * OVERSAMPLE;  // 0,1, 2,3, 4,5, ...
                out = _mm_add_pd(out, _mm_mul_pd(kernelPhase[p][j], buf[-offset]));
            }
        }

        return out;
    }
};



template <int OVERSAMPLE, int QUALITY>
struct Upsampler_128d {

    alignas(16) __m128d buffer[2 * QUALITY];
    alignas(16) __m128d kernelPhase[OVERSAMPLE][QUALITY];

    __m128d os;

    int inIndex;

    Upsampler_128d(double cutoff = 0.9) {
        constexpr int N = OVERSAMPLE * QUALITY;
        os = _mm_set1_pd((double)OVERSAMPLE);

        float kernel_f[N];
        rack::dsp::boxcarLowpassIR(kernel_f, N, (float)(cutoff * 0.5 / OVERSAMPLE));
        rack::dsp::blackmanHarrisWindow(kernel_f, N);

        for (int i = 0; i < OVERSAMPLE; i++) {
            for (int j = 0; j < QUALITY; j++) {
                int k = OVERSAMPLE * j + i;
                kernelPhase[i][j] = _mm_set1_pd((double)kernel_f[k]);
            }
        }

        reset();
    }

    void reset() {
        inIndex = 0;
        for (int i = 0; i < 2 * QUALITY; i++)
            buffer[i] = _mm_setzero_pd();
    }

    void process(__m128d in, __m128d* out) {
        __m128d scaled = _mm_mul_pd(os, in);

        // write + mirror
        buffer[inIndex] = scaled;
        buffer[inIndex + QUALITY] = scaled;

        inIndex++;
        if (inIndex >= QUALITY)
            inIndex -= QUALITY;

        __m128d* buf = &buffer[inIndex + QUALITY - 1];

        for (int i = 0; i < OVERSAMPLE; i++) {
            __m128d y = _mm_setzero_pd();
            for (int j = 0; j < QUALITY; j++) {
                y = _mm_add_pd(y, _mm_mul_pd(kernelPhase[i][j], buf[-j]));
            }

            out[i] = y;
        }
    }
};


} // namespace Omega::dsp
