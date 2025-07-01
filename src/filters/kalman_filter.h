/*

    Credit to Denys Sene, January, 1, 2017. I mostly just reused and refactored his 
    code specifically the filter estimation algorithm.
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

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>
#include <math.h>

class KalmanFilter {
    public:
        // Constructor with default uncertainties
        // e_mea: measurement uncertainty (>0)
        // e_est: estimation uncertainty (≥0)
        // q    : process noise (≥0)
        explicit KalmanFilter(float e_mea = 1.0f,
                              float e_est = 1.0f,
                              float q_    = 0.01f)
            :   _err_measure(e_mea),
                _err_estimate(e_est),
                _q(q_)
            { }

            // Set filter parameters at runtime
            bool setParameters(float e_mea, float e_est, float q) {
                if (e_mea <= 0.0f || e_est < 0.0f || q < 0.0f) {
                    return false;
                }

                _err_measure  = e_mea;
                _err_estimate = e_est;
                _q            = q;

                // (Re-initialize state if desired)
                _last_estimate    = 0.0f;
                _current_estimate = 0.0f;
                _kalman_gain      = 0.0f;

                return true;
            }

            // Update filter with a new measurement
            // Returns the filtered estimate
            float update(float measurement) {
                _kalman_gain        = _err_estimate / (_err_estimate + _err_measure);
                _current_estimate   = _last_estimate + _kalman_gain * (measurement - _last_estimate);
                _err_estimate       = (1.0f - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
                _last_estimate      = _current_estimate;

                return _current_estimate;
            }

    private:
        float _err_measure;
        float _err_estimate;
        float _q;
        float _current_estimate = 0;
        float _last_estimate    = 0;
        float _kalman_gain      = 0;
};

#endif 