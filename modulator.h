#pragma once
#include "vco.h"

namespace dsp::modulator {
    class r2FSKmodulator {
    public:
        r2FSKmodulator(float f0, float f1, float baudrate, int samplerate) {
            f1val = f1 / f0;
            vco.changeBaseFreq(f0, samplerate);
            symbolSamples = (1 / baudrate) * samplerate;
        }

        void changeParameters(float f0, float f1, float baudrate, int samplerate) {
            f1val = f1 / f0;
            vco.changeBaseFreq(f0, samplerate);
            symbolSamples = (1 / baudrate) * samplerate;
        }

        ~r2FSKmodulator() {
        }
        
        int calcOutSamples(int inCount) {
            return (inCount * 8) * symbolSamples;
        }

        // Expects raw bytes on the input
        void process(char* in, int inCount, float* out) {
            for(int i = 0; i < inCount; i++) {
                sampleBuffer = &out[i * (8 * symbolSamples)];
                for(int j = 0; j < 8; j++) {
                    float vcoVolt = 1;
                    if((in[i] >> j) & 0x01) {
                        vcoVolt = f1val;
                    }
                    for(int k = 0; k < symbolSamples; k++) {
                        sampleBuffer[(j * symbolSamples) + k] = vcoVolt;
                    }
                }
                vco.process(sampleBuffer, sampleBuffer, symbolSamples * 8);
            }
        }
    private:
        dsp::vco::rvco vco;
        float f1val = 0;
        float* sampleBuffer;
        unsigned int symbolSamples = 0; // the output baudrate may change slightly depending on what baudrate-samplerate combination is chosen! _this should definitely be fixed later_
    };
}