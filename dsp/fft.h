#pragma once

#include "window.h"
#include <complex>
#include <fftw3.h>
#include <volk/volk.h>

namespace dsp::fft {
    class complexPowerSpectrum {
    public:
        complexPowerSpectrum(int N) {
            _N = N;
            fft_window = (float *)fftwf_malloc(sizeof(float) * _N);
            for (int i = 0; i < _N; i++) {
                fft_window[i] = windowfunctions::blackman(i, _N - 1);
            }
            fftin = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * _N);
            fftout = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * _N);
            fftplan = fftwf_plan_dft_1d(_N, fftin, fftout, FFTW_FORWARD, FFTW_ESTIMATE);
        }

        ~complexPowerSpectrum() {
            fftwf_free(fftin);
            fftwf_free(fftout);
            fftwf_free(fft_window);
            fftwf_destroy_plan(fftplan);
        }

        void processFFT(std::complex<float> *in, float *out) {
            volk_32fc_32f_multiply_32fc((lv_32fc_t *)fftin, in, fft_window, _N);
            fftwf_execute(fftplan);
            volk_32fc_s32f_power_spectrum_32f(out, (lv_32fc_t *)fftout, _N, _N);
        }

    private:
        fftwf_complex *fftin;
        fftwf_complex *fftout;
        fftwf_plan fftplan;
        float *fft_window;
        int _N;
    };
}
