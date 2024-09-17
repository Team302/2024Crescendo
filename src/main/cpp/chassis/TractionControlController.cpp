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

    if (!isEnabled())
        return velocityOutput;

    inertialVelocity = std::abs(inertialVelocity);
    double currentSlipRatio = std::abs(
        inertialVelocity <= INERTIAL_VELOCITY_THRESHOLD
            ? wheelSpeed / m_maxLinearSpeed
            : (std::abs(wheelSpeed) - inertialVelocity) / inertialVelocity);
    m_isSlipping = m_slippingDebouncer.calculate(
        currentSlipRatio > m_optimalSlipRatio &&
        std::abs(wheelSpeed) > m_maxLinearSpeed * m_optimalSlipRatio &&
        isEnabled());

    double desiredAcceleration = (velocityRequest - inertialVelocity) / GlobalConstants::ROBOT_LOOP_PERIOD;

    double sigmoid = 1 / (1 + std::exp(-SIGMOID_K * std::clamp(2 * (currentSlipRatio - m_optimalSlipRatio) - 1, -1.0, 1.0)));

    double effectiveCoF = m_isSlipping ? m_staticCoF * (1 - sigmoid) + m_dynamicCoF * sigmoid : m_staticCoF;

    double predictedSlipRatio = std::abs(
                                    desiredAcceleration /
                                    (inertialVelocity * GlobalConstants::GRAVITATIONAL_ACCELERATION + effectiveCoF * m_mass * GlobalConstants::GRAVITATIONAL_ACCELERATION)) /
                                m_maxPredictedSlipRatio;

    double velocityCorrection = (m_optimalSlipRatio - predictedSlipRatio) * VELOCITY_CORRECTION_SCALAR * m_state;

    velocityOutput = std::clamp(velocityOutput + velocityCorrection, -m_maxLinearSpeed, m_maxLinearSpeed);

    return velocityOutput;
}

bool TractionControlController::isSlipping() const
{
    return m_isSlipping;
}

void TractionControlController::toggleTractionControl()
{
    m_state = static_cast<State>(!m_state);
}

void TractionControlController::enableTractionControl()
{
    m_state = State::ENABLED;
}

void TractionControlController::disableTractionControl()
{
    m_state = State::DISABLED;
}

bool TractionControlController::isEnabled() const
{
    return m_state == State::ENABLED;
}
