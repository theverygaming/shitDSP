#pragma once
#include "math.h"

namespace dsp::vco
{
    class rvco
    {
    public:
        rvco() {
        }

        rvco(float basefreq, float samplerate) {
            speed = ((FL_M_PI * 2) * basefreq) / samplerate;
        }

        ~rvco() {

        }

        void changeBaseFreq(float basefreq, float samplerate) {
            speed = ((FL_M_PI * 2) * basefreq) / samplerate;
        }

        void process(float *input, float *output, int count) {
            float in = 0; // this has been added to be able to use the same array as input and output
            for (int i = 0; i < count; i++) {
                in = input[i];
                output[i] = sin(phase);
                phase += in * speed;
            }
        }
    private:
        float phase = 0;
        float speed;
    };
}