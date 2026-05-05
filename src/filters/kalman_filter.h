/*

    Credit to Denys Sene, January, 1, 2017. I mostly just reused and refactored his 
    code specifically the filter estimation algorithm.
___  ___ _____ _____  ____  __   __   ___  ____ _         ______      __           _                     _ 
|  \/  |/  ___|  ___|/ ___|/  | /  |  |  \/  (_) |        | ___ \    / _|         | |                   | |
| .  . |\ `--.|___ \/ /___ `| | `| |  | .  . |_| | _____  | |_/ /___| |_ __ _  ___| |_ ___  _ __ ___  __| |
| |\/| | `--. \   \ \ ___ \ | |  | |  | |\/| | | |/ / _ \ |    // _ \  _/ _` |/ __| __/ _ \| '__/ _ \/ _` |
| |  | |/\__/ /\__/ / \_/ |_| |__| |_ | |  | | |   <  __/ | |\ \  __/ || (_| | (__| || (_) | | |  __/ (_| |
\_|  |_/\____/\____/\_____/\___/\___/ \_|  |_/_|_|\_\___| \_| \_\___|_| \__,_|\___|\__\___/|_|  \___|\__,_|
                              LIBRARY VERSION: 1.2.5_exp_build_04126026  

    Copyright (c) 2025 Francis Mike John Camogao
    Released under MIT License - see LICENSE file for details.
    GIT: https://github.com/mikeedudee/MS5611-Mike-Refactored.git

*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>
#include <math.h>

class KalmanFilter {
    public:
        explicit KalmanFilter(float e_mea = 1.0f,
                              float e_est = 1.0f,
                              float q_    = 0.01f)
            :   _err_measure(e_mea),
                _err_estimate(e_est),
                _q(q_),
                _initialized(false),
                _reject_count(0) { }

        bool setParameters(float e_mea, float e_est, float q) {
            if (e_mea <= 0.0f || e_est < 0.0f || q < 0.0f) {
                return false;
            }
            _err_measure  = e_mea;
            _err_estimate = e_est;
            _q            = q;
            return true;
        }

        // Standard Update
        float update(float measurement) {
            // First run initialization
            if (!_initialized) {
                _last_estimate = measurement;
                _current_estimate = measurement;
                _initialized = true;
                return measurement;
            }

            _kalman_gain        = _err_estimate / (_err_estimate + _err_measure);
            _current_estimate   = _last_estimate + _kalman_gain * (measurement - _last_estimate);
            _err_estimate       = (1.0f - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
            _last_estimate      = _current_estimate;

            return _current_estimate;
        }

        // Gated Update (Feature #4)
        // measurement: New raw data
        // sigma_gate:  How many standard deviations allowed? (Recommended: 3.0)
        // max_rejects: How many bad samples before we force an update? (Recommended: 10)
        float updateGated(float measurement, float sigma_gate, uint8_t max_rejects) {
            if (!_initialized) {
                return update(measurement);
            }

            // 1. Calculate Innovation (Residual)
            float innovation = measurement - _last_estimate;

            // 2. Calculate Theoretical Uncertainty (Standard Deviation of Innovation)
            // S = P + R (Estimate Error + Measurement Error)
            float innovation_uncertainty = sqrtf(_err_estimate + _err_measure);

            // 3. Gate Test: Is the error within N sigmas?
            if (fabsf(innovation) > (sigma_gate * innovation_uncertainty)) {
                // OUTLIER DETECTED
                _reject_count++;

                // If the signal stays "bad" for too long, it's not noise—it's real movement.
                // Force an update to catch up.
                if (_reject_count > max_rejects) {
                    _reject_count = 0;
                    return update(measurement);
                }

                // Reject update, return previous trusted estimate
                return _last_estimate;
            }

            // DATA IS VALID
            _reject_count = 0;
            return update(measurement);
        }

        void reset() {
            _initialized  = false;
            _reject_count = 0;
        }

    private:
        float _err_measure;
        float _err_estimate;
        float _q;
        float _current_estimate = 0.0f;
        float _last_estimate    = 0.0f;
        float _kalman_gain      = 0.0f;
        
        bool    _initialized;
        uint8_t _reject_count;
};

#endif