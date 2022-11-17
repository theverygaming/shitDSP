#pragma once
#include <cstring>
#include <fstream>

namespace dsp::wav {
    class wavWriter {
        // Epicly copied from https://github.com/AlexandreRouma/SDRPlusPlus/blob/master/misc_modules/recorder/src/wav.h
    public:
        wavWriter(std::string path, uint16_t bitDepth, uint16_t channelCount, uint32_t sampleRate) {
            outstream = std::ofstream(path.c_str(), std::ios::binary);
            header.formatHeaderLength = 16;
            header.sampleType = 1;
            if (bitDepth == 32) { // 32-bit is usually float
                header.sampleType = 3;
            }
            header.channelCount = channelCount;
            header.sampleRate = sampleRate;
            header.bytesPerSecond = sampleRate * channelCount * (bitDepth / 8);
            header.bytesPerSample = (bitDepth / 8) * channelCount;
            header.bitDepth = bitDepth;
            outstream.write((char *)&header, sizeof(waveheader));
        }

        ~wavWriter() {
            if (outstream.is_open()) {
                finish();
            }
        }

        void finish() {
            header.fileSize = written + sizeof(waveheader) - 8;
            header.dataSize = written;
            outstream.seekp(0);
            outstream.write((char *)&header, sizeof(waveheader));
            outstream.close();
        }

        bool isOpen() {
            return outstream.is_open();
        }

        void writeData(void *data, size_t size) {
            outstream.write((char *)data, size);
            written += size;
        }

    private:
        struct waveheader {
            char signature[4] = {'R', 'I', 'F', 'F'};
            uint32_t fileSize;
            char fileType[4] = {'W', 'A', 'V', 'E'};
            char formatMarker[4] = {'f', 'm', 't', ' '};
            uint32_t formatHeaderLength;
            uint16_t sampleType;
            uint16_t channelCount;
            uint32_t sampleRate;
            uint32_t bytesPerSecond;
            uint16_t bytesPerSample;
            uint16_t bitDepth;
            char dataMarker[4] = {'d', 'a', 't', 'a'};
            uint32_t dataSize;
        };

        waveheader header;
        std::ofstream outstream;
        size_t written = 0;
    };

    class wavReader {
        // https://github.com/AlexandreRouma/SDRPlusPlus/blob/master/source_modules/file_source/src/wavreader.h
    public:
        wavReader(std::string path) {
            instream = std::ifstream(path.c_str(), std::ios::binary);
            instream.seekg(0, instream.end);
            size_t filesize = instream.tellg();
            instream.seekg(0, instream.beg);
            instream.read((char *)&header, sizeof(waveheader));
            actualSampleCount = (filesize - sizeof(waveheader)) / (header.bitDepth / 8);
            if (memcmp(header.signature, "RIFF", 4) == 0 && memcmp(header.fileType, "WAVE", 4) == 0) {
                headerValid = true;
            }
        }

        ~wavReader() {
            if (instream.is_open()) {
                instream.close();
            }
        }

        bool isFileOpen() {
            return instream.is_open();
        }

        bool isHeaderValid() {
            return headerValid;
        }

        uint32_t getSamplerate() {
            return header.sampleRate;
        }

        uint16_t getBitDepth() {
            return header.bitDepth;
        }

        uint32_t getHeaderSampleCount() {
            return header.dataSize / header.bytesPerSample;
        }

        uint16_t getChannelCount() {
            return header.channelCount;
        }

        uint64_t getActualSampleCount() {
            return actualSampleCount;
        }

        void jumpToStart() {
            instream.seekg(sizeof(waveheader));
        }

        void readRaw(void *data, size_t size) {
            instream.read((char *)data, size);
        }

        void close() {
            instream.close();
        }

        bool readFloat(float *samples, unsigned long samplecount) {
            // float
            if (header.bitDepth == 32 && header.sampleType == 3) {
                readRaw((void *)samples, samplecount * sizeof(float));
                return true;
            }

            // 16-bit signed integer
            if (header.bitDepth == 16 && header.sampleType == 1) {
                int16_t *buffer = (int16_t *)malloc(samplecount * sizeof(int16_t));
                readRaw((void *)buffer, samplecount * sizeof(int16_t));
                for (unsigned long i = 0; i < samplecount; i++) {
                    samples[i] = buffer[i] / 32767.0f;
                }
                free(buffer);
                return true;
            }
            return false;
        }

    private:
        struct waveheader {
            char signature[4];
            uint32_t fileSize;
            char fileType[4];
            char formatMarker[4];
            uint32_t formatHeaderLength;
            uint16_t sampleType;
            uint16_t channelCount;
            uint32_t sampleRate;
            uint32_t bytesPerSecond;
            uint16_t bytesPerSample;
            uint16_t bitDepth;
            char dataMarker[4];
            uint32_t dataSize;
        };

        std::ifstream instream;
        waveheader header;
        bool headerValid = false;
        uint64_t actualSampleCount = 0;
    };
}
