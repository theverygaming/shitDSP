#pragma once
#include "firfilters.h"
#include <numeric>

namespace dsp
{
    namespace resamplers
    {
        class realUpsampler
        {
        public:
            realUpsampler(int multiplier, int chunkSize, int taps)
            {
                _multiplier = multiplier;
                _chunkSize = chunkSize;
                coeffs = (float *)malloc(taps * sizeof(float));
                filters::FIRcoeffcalc::calcCoeffs(dsp::filters::FIRcoeffcalc::lowpass, coeffs, taps, 48000, 24000 * ((float)1 / multiplier));
                lpf = new filters::FIRfilter(taps, coeffs, _chunkSize * multiplier);
                processarr = (float *)malloc(chunkSize * _multiplier * sizeof(float));
            }

            ~realUpsampler()
            {
                free(coeffs);
                free(processarr);
                delete lpf;
            }

            void upsample(int incount, float *in, float *out)
            {
                for (long int i = 0; i < incount; i++)
                {
                    processarr[i * _multiplier] = in[i];
                }
                lpf->filter(processarr, out, incount * _multiplier);
            }

        private:
            filters::FIRfilter *lpf;
            float *coeffs;
            float *processarr;
            int _multiplier;
            int _chunkSize;
        };
        class complexUpsampler
        {
        public:
            complexUpsampler(int chunkSize, int multiplier, int taps)
            {
                _chunkSize = chunkSize;
                _multiplier = multiplier;

                realInArr = (float *)calloc(_chunkSize, sizeof(float));
                imagInArr = (float *)calloc(_chunkSize, sizeof(float));

                realOutArr = (float *)malloc(_chunkSize * _multiplier * sizeof(float));
                imagOutArr = (float *)malloc(_chunkSize * _multiplier * sizeof(float));

                upReal = new resamplers::realUpsampler(_multiplier, _chunkSize, taps);
                upImag = new resamplers::realUpsampler(_multiplier, _chunkSize, taps);
            }

            ~complexUpsampler()
            {
                free(realInArr);
                free(imagInArr);

                free(realOutArr);
                free(imagOutArr);

                delete upReal;
                delete upImag;
            }

            void processSamples(std::complex<float> *in, std::complex<float> *out)
            {
                for (int i = 0; i < _chunkSize; i++)
                {
                    realInArr[i] = in[i].real();
                    imagInArr[i] = in[i].imag();
                }
                upReal->upsample(_chunkSize, realInArr, realOutArr);
                upImag->upsample(_chunkSize, imagInArr, imagOutArr);
                for (int i = 0; i < _chunkSize * _multiplier; i++)
                {
                    out[i] = {realOutArr[i], imagOutArr[i]};
                }
            }

        private:
            int _chunkSize;
            int _multiplier;
            float *realInArr;
            float *imagInArr;
            float *realOutArr;
            float *imagOutArr;
            resamplers::realUpsampler *upReal;
            resamplers::realUpsampler *upImag;
        };
        class realDownsampler
        {
        public:
            realDownsampler(int divider)
            {
                _divider = divider;
            }

            ~realDownsampler()
            {
            }

            void downsample(int incount, float *in, float *out)
            {
                for (int i = 0; i < incount; i += _divider)
                {
                    out[i / _divider] = in[i];
                }
            }

        private:
            int _divider;
        };

        class PSK_PulseShaping_CCRationalResamplerBlock // satdump copypasted 
        {
            public:
            PSK_PulseShaping_CCRationalResamplerBlock(int tapcount, unsigned int interpolation, float alpha, int maxInputSamples) {
                unsigned int decimation = 1;
                d_interpolation = interpolation;
                d_decimation = decimation;

                tapcount |= 1; // make sure tapcount is odd
                
                float *tapsr = (float*)malloc(tapcount * sizeof(float));
                filters::FIRcoeffcalc::root_raised_cosine(1, interpolation, decimation, alpha, tapcount, tapsr);

                int align = volk_get_alignment();

                // Buffer
                in_buffer = 0;
                int size = 2 * maxInputSamples;
                buffer = (std::complex<float> *)volk_malloc(size * sizeof(std::complex<float>), align);
                std::fill(buffer, &buffer[size], 0);

                // Start by reducing the interp and decim by their GCD
                int gcd = std::gcd(interpolation, decimation);
                d_interpolation /= gcd;
                d_decimation /= gcd;
                
                // Filter number & tap number
                nfilt = d_interpolation;
                ntaps = tapcount / nfilt;

                // If Ntaps is slightly over 1, add 1 tap
                if (fmod(double(tapcount) / double(nfilt), 1.0) > 0.0)
                    ntaps++;

                // Init tap buffers
                taps = (float **)volk_malloc(nfilt * sizeof(float *), align);
                for (int i = 0; i < nfilt; i++)
                {
                    taps[i] = (float *)volk_malloc(ntaps * sizeof(float), align);
                    memset(taps[i], 0, ntaps);
                }

                // Setup taps
                for (int i = 0; i < tapcount; i++)
                    taps[i % nfilt][(ntaps - 1) - (i / nfilt)] = tapsr[i];



                free(tapsr);
            }

            ~PSK_PulseShaping_CCRationalResamplerBlock() {
                for (int i = 0; i < nfilt; i++)
                    volk_free(taps[i]);
                volk_free(taps);
                volk_free(buffer);
            }

            void process(std::complex<float> *input, std::complex<float> *output, int nsamples) {
                memcpy(&buffer[ntaps], input, nsamples * sizeof(std::complex<float>));
                in_buffer = ntaps + nsamples;

                int outc = 0;
                for (int i = 0; i < in_buffer - ntaps;)
                {
                    volk_32fc_32f_dot_prod_32fc((lv_32fc_t *)&output[outc++], (lv_32fc_t *)&buffer[i], taps[d_ctr], ntaps);
                    
                    d_ctr += d_decimation;
                    while (d_ctr >= d_interpolation)
                    {
                        d_ctr -= d_interpolation;
                        i++;
                    }
                }

                memmove(&buffer[0], &buffer[in_buffer - ntaps], ntaps * sizeof(std::complex<float>));
            }

            
            private:
            // Settings
            int d_interpolation;
            int d_decimation;
            int d_ctr = 0;

            // Buffer
            std::complex<float> *buffer;
            int in_buffer;

            // Taps
            float **taps;
            int nfilt; // Number of filters (one per phase)
            int ntaps;
        };
    }
}