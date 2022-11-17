#pragma once
#include "math.h"
#include "window.h"
#include <string.h>
#include <vector>
#include <volk/volk.h>

namespace dsp {
    namespace filters {
        class FIRcoeffcalc {
        public:
            enum filter_type { lowpass, highpass, bandpass, bandstop };

            static std::vector<float> calcCoeffs(filter_type type, int taps, int samplerate, float frequency) {
                std::vector<float> impulseresponse;
                impulseresponse.resize(taps);
                float sampletime = 1 / (float)samplerate;
                for (int i = 0; i < taps; i++) {
                    int x = i - (taps / 2);
                    if (x == 0) {
                        switch (type) {
                        case lowpass:
                            impulseresponse[i] = 2 * frequency;
                            break;
                        case highpass:
                            break;
                        }
                        continue;
                    }
                    switch (type) {
                    case lowpass:
                        impulseresponse[i] = sin(2 * FL_M_PI * frequency * sampletime * x) / (FL_M_PI * sampletime * x);
                        break;
                    case highpass:
                        break;
                    }
                }
                for (int i = 0; i < taps; i++) {
                    // impulseresponse[i] *= sampletime;
                    impulseresponse[i] /= frequency * 2;
                    impulseresponse[i] *= windowfunctions::blackman(i, taps - 1);
                }
                return impulseresponse;
            }

            static std::vector<float> calcCoeffs_band(filter_type type, int taps, int samplerate, float freq1, float freq2) {
                std::vector<float> impulseresponse;
                impulseresponse.resize(taps);
                float sampletime = 1 / (float)samplerate;
                for (int i = 0; i < taps; i++) {
                    int x = i - (taps / 2);
                    if (x == 0) {
                        switch (type) {
                        case bandpass:
                            impulseresponse[i] = 2 * freq2 - 2 * freq1;
                            break;
                        case bandstop:
                            break;
                        }
                        continue;
                    }
                    switch (type) {
                    case bandpass:
                        impulseresponse[i] = (sin(2 * FL_M_PI * freq2 * sampletime * x) - sin(2 * FL_M_PI * freq1 * sampletime * x)) / (FL_M_PI * sampletime * x);
                        break;
                    case bandstop:
                        break;
                    }
                }
                for (int i = 0; i < taps; i++) {
                    impulseresponse[i] *= sampletime;
                    // impulseresponse[i] /= 2 * freq2 - 2 * freq1;
                    impulseresponse[i] *= windowfunctions::blackman(i, taps - 1);
                }
                return impulseresponse;
            }

            static void root_raised_cosine(double gain, double sampling_freq, double symbol_rate, double alpha, int ntaps, float *taps) // from SatDump
            {
                // ntaps |= 1; // ensure that ntaps is odd

                double spb = sampling_freq / symbol_rate; // samples per bit/symbol

                double scale = 0;
                for (int i = 0; i < ntaps; i++) {
                    double x1, x2, x3, num, den;
                    double xindx = i - ntaps / 2;
                    x1 = M_PI * xindx / spb;
                    x2 = 4 * alpha * xindx / spb;
                    x3 = x2 * x2 - 1;

                    if (fabs(x3) >= 0.000001) { // Avoid Rounding errors...
                        if (i != ntaps / 2)
                            num = cos((1 + alpha) * x1) + sin((1 - alpha) * x1) / (4 * alpha * xindx / spb);
                        else
                            num = cos((1 + alpha) * x1) + (1 - alpha) * M_PI / (4 * alpha);
                        den = x3 * M_PI;
                    } else {
                        if (alpha == 1) {
                            taps[i] = -1;
                            scale += taps[i];
                            continue;
                        }
                        x3 = (1 - alpha) * x1;
                        x2 = (1 + alpha) * x1;
                        num = (sin(x2) * (1 + alpha) * M_PI - cos(x3) * ((1 - alpha) * M_PI * spb) / (4 * alpha * xindx) + sin(x3) * spb * spb / (4 * alpha * xindx * xindx));
                        den = -32 * M_PI * alpha * alpha * xindx / spb;
                    }
                    taps[i] = 4 * alpha * num / den;
                    scale += taps[i];
                }

                for (int i = 0; i < ntaps; i++)
                    taps[i] = taps[i] * gain / scale;
            }

        private:
        };

        class FIRfilter {
        public:
            FIRfilter(std::vector<float> taps, int chunkSize) {
                _taps = taps;
                buffer = (float *)malloc(((chunkSize * 2) + _taps.size()) * sizeof(float));
                bufferStart = &buffer[_taps.size() - 1];
            }

            ~FIRfilter() {
                free(buffer);
            }

            void run(float *in, float *out, size_t count) {
                memcpy(bufferStart, in, count * sizeof(float));
                for (size_t i = 0; i < count; i++) {
                    volk_32f_x2_dot_prod_32f(&out[i], &buffer[i], _taps.data(), _taps.size());
                }
                memmove(buffer, &buffer[count], (_taps.size() - 1) * sizeof(float));
            }

        private:
            std::vector<float> _taps;
            float *buffer;
            float *bufferStart;
        };
    }
}
