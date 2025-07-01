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

#ifndef DERIVATIVE_ESTIMATOR_H
#define DERIVATIVE_ESTIMATOR_H

#include <Arduino.h>  // for micros()

/// Simple first‐order derivative estimator (for velocity or acceleration)
class DerivativeEstimator {
public:
    DerivativeEstimator() : _initialized(false), _lastTime(0.0f), _lastValue(0.0f) {}

    float update(float value, float time) {
        if (!_initialized) {
            _initialized = true;
            _lastTime    = time;
            _lastValue   = value;

            return 0.0f;
        }

        // Ensure time is monotonic and greater than last recorded time
        float dt = time - _lastTime;
        
        if (dt <= 0.0f) {
            return 0.0f;  // Protect against non-monotonic or zero interval
        }

        float derivative    = (value - _lastValue) / dt;
        _lastTime           = time;
        _lastValue          = value;

        return derivative;
    }

    /// Reset the estimator to its uninitialized state
    void reset() {
        _initialized = false;
    }

private:
    bool    _initialized;
    float   _lastTime;
    float   _lastValue;
};

#endif  // DERIVATIVE_ESTIMATOR_H
