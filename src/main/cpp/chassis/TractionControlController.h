//====================================================================================================================================================
// Copyright 2024 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes
#include <vector>
#include <cmath>

// FRC Includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/time.h"
#include "frc/filter/Debouncer.h"

// Team 302 Includes

using namespace frc;

class TractionControlController
{
public:
    TractionControlController(double staticCoF, double dynamicCoF, double optimalSlipRatio, double mass, units::velocity::meters_per_second_t maxLinearSpeed);

    units::velocity::meters_per_second_t calculate(units::velocity::meters_per_second_t velocityRequest, units::velocity::meters_per_second_t inertialVelocity, units::velocity::meters_per_second_t wheelSpeed);
    bool isSlipping() const;

private:
    static constexpr double VELOCITY_CORRECTION_SCALAR = 0.7;
    static constexpr double MIN_SLIP_RATIO = 0.01;
    static constexpr double MAX_SLIP_RATIO = 0.40;
    static constexpr int SIGMOID_K = 10;
    static constexpr units::velocity::meters_per_second_t INERTIAL_VELOCITY_THRESHOLD{0.01};
    static constexpr double m_robotLoopRate_Hz = 50.0;

    units::second_t MIN_SLIPPING_TIME{1.1};

    double m_staticCoF;
    double m_dynamicCoF;
    double m_optimalSlipRatio;
    double m_mass;
    units::velocity::meters_per_second_t m_maxLinearSpeed;
    double m_maxPredictedSlipRatio;
    bool m_isSlipping;
    frc::Debouncer m_slippingDebouncer;
};
