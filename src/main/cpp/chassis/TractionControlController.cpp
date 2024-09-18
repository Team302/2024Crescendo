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
// This file was automatically generated by the Team 302 code generator version 1.3.0.8
// Generated on Wednesday, February 21, 2024 8:17:08 PM

// C++ Includes
#include "algorithm"

// FRC Includes

// Team 302 Includes
#include "chassis/TractionControlController.h"

TractionControlController::TractionControlController(double staticCoF,
                                                     double dynamicCoF,
                                                     double optimalSlipRatio,
                                                     double mass,
                                                     double maxLinearSpeed) : m_staticCoF(staticCoF),
                                                                              m_dynamicCoF(dynamicCoF),
                                                                              m_optimalSlipRatio(optimalSlipRatio),
                                                                              m_mass(mass / 4),
                                                                              m_maxLinearSpeed(maxLinearSpeed),
                                                                              m_maxPredictedSlipRatio((maxLinearSpeed * m_robotLoopRate_Hz) / (staticCoF * m_mass * 9.81)),
                                                                              m_isSlipping(false), m_slippingDebouncer(MIN_SLIPPING_TIME, frc::Debouncer::DebounceType::kRising), m_state(State::ENABLED)
{
    if (dynamicCoF > staticCoF)
    {
        throw std::invalid_argument("Static CoF must be higher than dynamic CoF!");
    }
    m_optimalSlipRatio = std::clamp(optimalSlipRatio, MIN_SLIP_RATIO, MAX_SLIP_RATIO);
}

double TractionControlController::calculate(double velocityRequest, double inertialVelocity, double wheelSpeed)
{
    double velocityOutput = velocityRequest;

    inertialVelocity = std::abs(inertialVelocity);
    double currentSlipRatio = std::abs(
        inertialVelocity <= INERTIAL_VELOCITY_THRESHOLD.value()
            ? wheelSpeed / m_maxLinearSpeed
            : (std::abs(wheelSpeed) - inertialVelocity) / inertialVelocity);
    m_isSlipping = m_slippingDebouncer.Calculate(
        currentSlipRatio > m_optimalSlipRatio &&
        std::abs(wheelSpeed) > m_maxLinearSpeed * m_optimalSlipRatio);

    double desiredAcceleration = (velocityRequest - inertialVelocity) / m_robotLoopRate_Hz;

    double sigmoid = 1 / (1 + std::exp(-SIGMOID_K * std::clamp(2 * (currentSlipRatio - m_optimalSlipRatio) - 1, -1.0, 1.0)));

    double effectiveCoF = m_isSlipping ? m_staticCoF * (1 - sigmoid) + m_dynamicCoF * sigmoid : m_staticCoF;

    double predictedSlipRatio = std::abs(
                                    desiredAcceleration /
                                    (inertialVelocity * 9.81 + effectiveCoF * m_mass * 9.81)) /
                                m_maxPredictedSlipRatio;

    double velocityCorrection = (m_optimalSlipRatio - predictedSlipRatio) * VELOCITY_CORRECTION_SCALAR * m_state;

    velocityOutput = std::clamp(velocityOutput + velocityCorrection, -m_maxLinearSpeed, m_maxLinearSpeed);

    return velocityOutput;
}

bool TractionControlController::isSlipping() const
{
    return m_isSlipping;
}
