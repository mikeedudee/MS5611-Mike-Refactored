/*
___  ___ _____ _____  ____  __   __   ___  ____ _         ______      __           _                     _ 
|  \/  |/  ___|  ___|/ ___|/  | /  |  |  \/  (_) |        | ___ \    / _|         | |                   | |
| .  . |\ `--.|___ \/ /___ `| | `| |  | .  . |_| | _____  | |_/ /___| |_ __ _  ___| |_ ___  _ __ ___  __| |
| |\/| | `--. \   \ \ ___ \ | |  | |  | |\/| | | |/ / _ \ |    // _ \  _/ _` |/ __| __/ _ \| '__/ _ \/ _` |
| |  | |/\__/ /\__/ / \_/ |_| |__| |_ | |  | | |   <  __/ | |\ \  __/ || (_| | (__| || (_) | | |  __/ (_| |
\_|  |_/\____/\____/\_____/\___/\___/ \_|  |_/_|_|\_\___| \_| \_\___|_| \__,_|\___|\__\___/|_|  \___|\__,_|
                              LIBRARY VERSION: 1.0.8_exp_build_01082025  

    Copyright (c) 2025 Francis Mike John Camogao
    Released under MIT License - see LICENSE file for details.
    GIT: https://github.com/mikeedudee/MS5611-Mike-Refactored.git

*/

#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdint.h>

// A fixed‐maximum median filter with runtime window‐size control
class MedianFilter {
    public:
        // Maximum allowed window size (must be odd)
        static constexpr uint8_t MAX_WINDOW = 21u;
        
        // Constructor: default window size = 5
        explicit MedianFilter(uint8_t ws = 5u) 
        : windowSize_(0), idx_(0), count_(0) 
        {
            static_assert((MAX_WINDOW % 2u) == 1u, "MAX_WINDOW must be odd");
            setWindowSize(ws);
        }

        // Set the window size at runtime (must be 1 ≤ ws ≤ MAX_WINDOW, odd)
        // Returns true if successful
        bool setWindowSize(uint8_t ws) {
            if (ws == 0u || ws > MAX_WINDOW || (ws % 2u) == 0u) {
                return false;
            }

            windowSize_ = ws;
            idx_        = 0u;
            count_      = 0u;

            // Zero‐initialize buffer
            for (uint8_t i = 0u; i < MAX_WINDOW; ++i) {
                buffer_[i] = 0.0f;
            }

            return true;
        }

        // Feed a new sample; returns the median of the last windowSize_ samples
        float update(float sample) {
            // Insert into circular buffer
            buffer_[idx_]   = sample;
            idx_            = (idx_ + 1u) % windowSize_;

            if (count_ < windowSize_) {
                ++count_;
            }

            // Copy valid samples into temp
            float temp[MAX_WINDOW];

            for (uint8_t i = 0u; i < count_; ++i) {
                temp[i] = buffer_[i];
            }

            // Insertion‐sort first count_ elements
            for (uint8_t i = 1u; i < count_; ++i) {
                float   key = temp[i];
                int8_t  j   = i - 1;
                
                while (j >= 0 && temp[j] > key) {
                    temp[j + 1u] = temp[j];
                    --j;
                }

                temp[uint8_t(j + 1u)] = key;
            }
            // Return middle element
            return temp[count_ / 2u];
        }

    private:
        uint8_t windowSize_;
        uint8_t idx_;
        uint8_t count_;
        float   buffer_[MAX_WINDOW];
};

#endif  // MEDIAN_FILTER_H
