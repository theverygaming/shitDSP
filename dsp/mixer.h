#pragma once
#include <cmath>

namespace dsp::mixer {
    class real_mixer {
    public:
        real_mixer(float frequency, float samplerate) {
            change_frequency(frequency, samplerate);
        }

        ~real_mixer() {}

        void change_frequency(float frequency, float samplerate) {
            _time = 1 / samplerate;
            _frequency = frequency;
        }

        void run(float *out, float *in, size_t count) {
            for (size_t i = 0; i < count; i++) {
                out[i] = in[i] * (sin(fmod(2 * M_PI * (_time * _count) * _frequency, 2 * M_PI)));
                _count++;
            }
        }

    private:
        size_t _count = 0;
        double _time;
        double _frequency;
    };
}
