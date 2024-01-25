
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

#include "configs/RobotElementNames.h"
#include "hw/DragonSparkMax.h"
#include "mechanisms/controllers/ControlData.h"

#include "frc/smartdashboard/SmartDashboard.h"
using rev::CANSparkMax;

DragonSparkMax::DragonSparkMax(int id,
                               RobotElementNames::MOTOR_CONTROLLER_USAGE deviceType,
                               CANSparkMax::MotorType motorType,
                               rev::SparkRelativeEncoder::Type feedbackType,
                               double gearRatio) : IDragonMotorController(),
                                                   m_id(id),
                                                   m_spark(new CANSparkMax(id, motorType)),
                                                   // m_controlMode(DRAGON_CONTROL_MODE::PERCENT_OUTPUT),
                                                   m_outputRotationOffset(0.0),
                                                   m_gearRatio(gearRatio),
                                                   m_deviceType(deviceType),
                                                   m_feedbackType(feedbackType),
                                                   m_encoder(m_spark->GetEncoder(m_feedbackType)),
                                                   m_pidController(m_spark->GetPIDController())
{
    m_spark->RestoreFactoryDefaults(true);

    m_pidController.SetOutputRange(-1.0, 1.0, 0);
    m_pidController.SetOutputRange(-1.0, 1.0, 1);
    m_spark->SetOpenLoopRampRate(0.09); // 0.2 0.25
    m_spark->SetClosedLoopRampRate(0.02);
    m_encoder.SetPosition(0);
    SetRotationOffset(0);
}

double DragonSparkMax::GetRotations()
{
    return GetRotationsWithGearNoOffset() - m_outputRotationOffset;
}

double DragonSparkMax::GetRPS()
{
    return m_encoder.GetVelocity() / 60.0;
}

RobotElementNames::MOTOR_CONTROLLER_USAGE DragonSparkMax::GetType() const
{
    return m_deviceType;
}

int DragonSparkMax::GetID() const
{
    return m_id;
}

void DragonSparkMax::SetControlConstants(int slot, const ControlData &controlInfo)
{
    m_pidController.SetP(controlInfo.GetP(), slot);
    m_pidController.SetI(controlInfo.GetI(), slot);
    m_pidController.SetD(controlInfo.GetD(), slot);
    m_pidController.SetFF(controlInfo.GetF(), slot);

    switch (controlInfo.GetMode())
    {
    case ControlModes::PERCENT_OUTPUT:
        m_spark->Set(0); // init to zero just to be safe
        break;

    case ControlModes::POSITION_INCH:
        m_pidController.SetReference(0, CANSparkMax::ControlType::kPosition, slot);
        break;

    case ControlModes::VELOCITY_RPS:
        m_pidController.SetReference(0, CANSparkMax::ControlType::kVelocity, slot);
        break;

    default:
        // danger11!!!!
        m_spark->Set(0);
        break;
    }
}

void DragonSparkMax::Set(double value)
{
    // TODO: need to fix
    /**
    switch (m_controlMode)
    {
        case DRAGON_CONTROL_MODE::PERCENT_OUTPUT:
            m_spark->Set(value);
            break;

        case DRAGON_CONTROL_MODE::ROTATIONS:
            // (rot * gear ratio) - m_outputRotationOffset
            m_spark->GetPIDController().SetReference((value + m_outputRotationOffset) / m_gearRatio, rev::ControlType::kPosition, 0); // position is slot 0
            break;

        case DRAGON_CONTROL_MODE::RPS: //inches per second
            m_spark->GetPIDController().SetReference((value / 60.0) / m_gearRatio, rev::ControlType::kVelocity, 1);
            break;

        default:
            // bad news if we are in the default branch... stop the motor
            m_spark->Set(0);
            break;
    }
    **/
    m_spark->Set(value);
}

void DragonSparkMax::SetRotationOffset(double rotations)
{
    m_outputRotationOffset = GetRotationsWithGearNoOffset() - rotations;
}

void DragonSparkMax::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    m_spark->SetOpenLoopRampRate(ramping);
    m_spark->SetClosedLoopRampRate(ramping); // TODO: should closed and open be separate

    if (rampingClosedLoop >= 0)
    {
        m_spark->SetClosedLoopRampRate(rampingClosedLoop);
    }
}

void DragonSparkMax::EnableCurrentLimiting(bool enabled)
{
    // TODO:
    // m_spark->SetSmart
}

void DragonSparkMax::EnableBrakeMode(bool enabled)
{
    m_spark->SetIdleMode(enabled ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
}

void DragonSparkMax::Invert(bool inverted)
{
    m_spark->SetInverted(inverted);
    // m_spark->GetEncoder().SetPositionConversionFactor(inverted ? -1.0 : 1.0);
}

double DragonSparkMax::GetRotationsWithGearNoOffset() const
{
    return m_encoder.GetPosition() * m_gearRatio;
}

void DragonSparkMax::InvertEncoder(bool inverted)
{
    // m_spark->SetInverted()
    // m_spark->GetEncoder().SetInverted(inverted);
}

CANSparkMax *DragonSparkMax::GetSparkMax()
{
    return m_spark;
}

void DragonSparkMax::SetSmartCurrentLimiting(int limit)
{
    m_spark->SetSmartCurrentLimit(limit);
}

// Dummy methods below
double DragonSparkMax::GetCurrent()
{
    return 0.0;
}
IDragonMotorController::MOTOR_TYPE DragonSparkMax::GetMotorType() const
{
    return IDragonMotorController::MOTOR_TYPE::NEO500MOTOR;
}

void DragonSparkMax::SetSensorInverted(bool inverted)
{
}
void DragonSparkMax::SetDiameter(double diameter)
{
}

void DragonSparkMax::SetVoltage(units::volt_t output)
{
    m_spark->SetVoltage(output);
}

bool DragonSparkMax::IsMotorInverted() const
{
}

bool DragonSparkMax::IsForwardLimitSwitchClosed()
{
    return false;
}

bool DragonSparkMax::IsReverseLimitSwitchClosed()
{
    return false;
}

void DragonSparkMax::EnableDisableLimitSwitches(
    bool enable)
{
}

void DragonSparkMax::EnableVoltageCompensation(double fullvoltage)
{
}

void DragonSparkMax::SetSelectedSensorPosition(
    double initialPosition)
{
}

double DragonSparkMax::GetCountsPerInch() const
{
    return 1.0;
}
double DragonSparkMax::GetCountsPerDegree() const
{
    return 1.0;
}

double DragonSparkMax::GetCounts()
{
    return m_spark->GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle).GetPosition();
}

void DragonSparkMax::SetRemoteSensor(int canID, ctre::phoenix::motorcontrol::RemoteSensorSource deviceType)
{
}
