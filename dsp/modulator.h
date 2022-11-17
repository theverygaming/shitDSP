#pragma once
#include "firfilters.h"
#include "math.h"
#include "vco.h"
#include <complex>

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
            fprintf(stderr, "dsp::modulator::r2FSKmodulator - actual baudrate: %f\n", (1 / (float)symbolSamples) * samplerate);
        }

        ~r2FSKmodulator() {}

        int calcOutSamples(int inCount) {
            return (inCount * 8) * symbolSamples;
        }

        // Expects raw bytes on the input
        void process(char *in, int inCount, float *out) {
            for (int i = 0; i < inCount; i++) {
                sampleBuffer = &out[i * (8 * symbolSamples)];
                for (int j = 0; j < 8; j++) {
                    float vcoVolt = 1;
                    if ((in[i] >> j) & 0x01) {
                        vcoVolt = f1val;
                    }
                    for (int k = 0; k < symbolSamples; k++) {
                        sampleBuffer[(j * symbolSamples) + k] = vcoVolt;
                    }
                }
                vco.process(sampleBuffer, sampleBuffer, symbolSamples * 8);
            }
        }

    private:
        dsp::vco::rvco vco;
        float f1val = 0;
        float *sampleBuffer;
        unsigned int symbolSamples = 0; // the output baudrate may change slightly depending on what baudrate-samplerate combination is chosen! _this should definitely be fixed later_
    };

    class cQPSKmodulator {
    public:
        cQPSKmodulator(float symrate, int samplerate) {
            changeParameters(symrate, samplerate);
        }

        void changeParameters(float symrate, int samplerate) {
            symbolSamples = (1 / symrate) * samplerate;
            fprintf(stderr, "dsp::modulator::cQPSKmodulator - actual symbolrate: %f\n", (1 / (float)symbolSamples) * samplerate);
        }

        ~cQPSKmodulator() {}

        int calcOutSamples(int inCount) {
            return ((inCount * 8) / 2) * symbolSamples;
        }

        // Expects raw bytes on the input
        void process(char *in, int inCount, std::complex<float> *out) {
            int counterSamples = 0;
            int a = 0;
            for (int i = 0; i < inCount; i++) {
                for (int j = 0; j < 4; j++) {

                    // differential encoder
                    unsigned int newPhaseIndex = ((unsigned int)getBitPair(in[i], j) + lastPhaseIndex) % 4;
                    lastPhaseIndex = newPhaseIndex;

                    float PhaseShiftDeg = (float)phases[newPhaseIndex];

                    float PhaseShiftRadians = PhaseShiftDeg * (FL_M_PI / 180);

                    float reVal = cos(PhaseShiftRadians);
                    float imVal = sin(PhaseShiftRadians);

                    for (int k = 0; k < symbolSamples; k++) {
                        out[counterSamples] = {reVal, imVal};
                        counterSamples++;
                    }
                }
            }
        }

    private:
        unsigned int symbolSamples = 0;

        unsigned char getBitPair(unsigned char in, int n) {
            return (in >> (n * 2)) & 0x3;
        }

        /*
         * +-------+-------+  +----+----+
         * | 135°  | 45°   |  | 01 | 11 |
         * +-------+-------+  +----+----+
         * | 225°  | 315°  |  | 00 | 10 |
         * +-------+-------+  +----+----+
         */
        int phases[4] = {45, 135, 225, 315};
        unsigned int lastPhaseIndex = 0;
    };

    class rFSKvcogen {
    public:
        rFSKvcogen(float symrate, int samplerate) {
            changeParameters(symrate, samplerate);
        }

        void changeParameters(float symrate, int samplerate) {
            symbolSamples = (1 / symrate) * samplerate;
            fprintf(stderr, "dsp::modulator::rFSKvcogen - actual symbolrate: %f\n", (1 / (float)symbolSamples) * samplerate);
        }

        ~rFSKvcogen() {}

        int calcOutSamples(int inCount) {
            return (inCount * 8) * symbolSamples;
        }

        // Expects raw bytes on the input
        void process(char *in, int inCount, float *out) {
            int counterSamples = 0;
            int a = 0;
            for (int i = 0; i < inCount; i++) {
                for (int j = 0; j < 8; j++) {

                    // differential encoder
                    unsigned int newIndex = ((unsigned int)getBit(in[i], j) + lastIndex) % 2;
                    lastIndex = newIndex;

                    float val = values[lastIndex];

                    for (int k = 0; k < symbolSamples; k++) {
                        out[counterSamples] = {val};
                        counterSamples++;
                    }
                }
            }
        }

    private:
        unsigned int symbolSamples = 0;

        unsigned char getBit(unsigned char in, int n) {
            return (in >> (n)) & 0x1;
        }

        float values[2] = {-1, 1};
        unsigned int lastIndex = 0;
    };
}
