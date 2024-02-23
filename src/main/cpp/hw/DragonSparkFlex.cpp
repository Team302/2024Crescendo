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
#include "hw/DragonSparkFlex.h"
#include "mechanisms/controllers/ControlData.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "utils/logging/Logger.h"
using rev::CANSparkFlex;

DragonSparkFlex::DragonSparkFlex(int id,
                                 RobotElementNames::MOTOR_CONTROLLER_USAGE deviceType,
                                 CANSparkFlex::MotorType motorType,
                                 rev::SparkRelativeEncoder::Type feedbackType,
                                 rev::SparkLimitSwitch::Type forwardType,
                                 rev::SparkLimitSwitch::Type reverseType,
                                 const DistanceAngleCalcStruc &calcStruc) : IDragonMotorController(),
                                                                            m_id(id),
                                                                            m_spark(new CANSparkFlex(id, motorType)),
                                                                            m_outputRotationOffset(0.0),
                                                                            m_deviceType(deviceType),
                                                                            m_feedbackType(feedbackType),
                                                                            m_forwardType(forwardType),
                                                                            m_reverseType(reverseType),
                                                                            m_encoder(m_spark->GetEncoder(m_feedbackType)),
                                                                            m_pidController(m_spark->GetPIDController()),
                                                                            m_forwardLimitSwitch(m_spark->GetForwardLimitSwitch(m_forwardType)),
                                                                            m_reverseLimitSwitch(m_spark->GetReverseLimitSwitch(m_reverseType)),
                                                                            m_calcStruc(calcStruc)
{
    m_spark->RestoreFactoryDefaults();
    m_pidController.SetOutputRange(-1.0, 1.0, 0);
    m_pidController.SetOutputRange(-1.0, 1.0, 1);
    m_spark->SetOpenLoopRampRate(0.09);
    m_spark->SetClosedLoopRampRate(0.02);
    m_encoder.SetPosition(0);
    SetRotationOffset(0);
    m_forwardLimitSwitch.EnableLimitSwitch(false);
    m_reverseLimitSwitch.EnableLimitSwitch(false);
}

double DragonSparkFlex::GetRotations()
{
    return GetRotationsWithGearNoOffset() - m_outputRotationOffset;
}

double DragonSparkFlex::GetRPS()
{
    return m_encoder.GetVelocity() / 60.0;
}

RobotElementNames::MOTOR_CONTROLLER_USAGE DragonSparkFlex::GetType() const
{
    return m_deviceType;
}

int DragonSparkFlex::GetID() const
{
    return m_id;
}

void DragonSparkFlex::SetControlConstants(int slot, const ControlData &controlInfo)
{
    m_pidController.SetP(controlInfo.GetP(), slot);
    m_pidController.SetI(controlInfo.GetI(), slot);
    m_pidController.SetD(controlInfo.GetD(), slot);
    m_pidController.SetFF(controlInfo.GetF(), slot);
    m_slot = slot;

    switch (controlInfo.GetMode())
    {
    case ControlModes::PERCENT_OUTPUT:
        // m_spark->Set(0); // init to zero just to be safe
        m_controlType = rev::CANSparkFlex::ControlType ::kDutyCycle;
        break;
    case ControlModes::POSITION_INCH:
        //  m_pidController.SetReference(0, CANSparkFlex::ControlType::kPosition, slot);
        m_encoder.SetPositionConversionFactor(m_calcStruc.countsPerInch);
        m_controlType = rev::CANSparkFlex::ControlType::kPosition;
        break;
    case ControlModes::POSITION_DEGREES:
        //   m_pidController.SetReference(0, CANSparkFlex::ControlType::kPosition, slot);
        m_encoder.SetPositionConversionFactor(m_calcStruc.countsPerDegree);
        m_controlType = rev::CANSparkFlex::ControlType ::kPosition;
        break;
    case ControlModes::VELOCITY_RPS:
        // m_pidController.SetReference(0, CANSparkFlex::ControlType::kVelocity, slot);
        m_encoder.SetVelocityConversionFactor(m_calcStruc.countsPerRev);
        m_controlType = rev::CANSparkFlex::ControlType::kVelocity;
        break;

    default:
        // danger11!!!!
        m_spark->Set(0);
        break;
    }
}

void DragonSparkFlex::EnableCurrentLimiting(bool enabled)
{
}

void DragonSparkFlex::Set(double value)
{
    if (m_controlType == rev::CANSparkFlex::ControlType::kDutyCycle)
    {
        m_spark->Set(value);
    }
    else
    {
        m_pidController.SetReference(value, m_controlType, m_slot);
    }
}

void DragonSparkFlex::SetRotationOffset(double rotations)
{
    m_outputRotationOffset = GetRotationsWithGearNoOffset() - rotations;
}

void DragonSparkFlex::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    m_spark->SetOpenLoopRampRate(ramping);
    m_spark->SetClosedLoopRampRate(ramping); // TODO: should closed and open be separate

    if (rampingClosedLoop >= 0)
    {
        m_spark->SetClosedLoopRampRate(rampingClosedLoop);
    }
}

void DragonSparkFlex::EnableBrakeMode(bool enabled)
{
    m_spark->SetIdleMode(enabled ? rev::CANSparkFlex::IdleMode::kBrake : rev::CANSparkFlex::IdleMode::kCoast);
}

void DragonSparkFlex::Invert(bool inverted)
{
    m_spark->SetInverted(inverted);
}

double DragonSparkFlex::GetRotationsWithGearNoOffset() const
{
    return m_encoder.GetPosition() * m_calcStruc.gearRatio;
}

void DragonSparkFlex::InvertEncoder(bool inverted)
{
    m_encoder.SetInverted(inverted);
}

CANSparkFlex *DragonSparkFlex::GetSparkFlex()
{
    return m_spark;
}

void DragonSparkFlex::SetSmartCurrentLimiting(int limit)
{
    m_spark->SetSmartCurrentLimit(limit);
}

void DragonSparkFlex::SetSecondaryCurrentLimiting(int limit, int duration)
{
    m_spark->SetSecondaryCurrentLimit(limit, duration);
}

double DragonSparkFlex::GetCurrent()
{
    return m_spark->GetOutputCurrent();
}
IDragonMotorController::MOTOR_TYPE DragonSparkFlex::GetMotorType() const
{
    return IDragonMotorController::MOTOR_TYPE::VORTEX;
}

void DragonSparkFlex::SetSensorInverted(bool inverted)
{
}
void DragonSparkFlex::SetDiameter(double diameter)
{
}

void DragonSparkFlex::SetVoltage(units::volt_t output)
{
    m_spark->SetVoltage(output);
}

bool DragonSparkFlex::IsMotorInverted() const
{
    return m_spark->GetInverted();
}

bool DragonSparkFlex::IsForwardLimitSwitchClosed()
{
    return m_forwardLimitSwitch.Get();
}

bool DragonSparkFlex::IsReverseLimitSwitchClosed()
{
    return m_reverseLimitSwitch.Get();
}

void DragonSparkFlex::EnableDisableLimitSwitches(bool enable)
{
    m_forwardLimitSwitch.EnableLimitSwitch(enable);
    m_reverseLimitSwitch.EnableLimitSwitch(enable);
}

void DragonSparkFlex::EnableVoltageCompensation(double fullvoltage)
{
    m_spark->EnableVoltageCompensation(fullvoltage);
}

void DragonSparkFlex::SetSelectedSensorPosition(
    double initialPosition)
{
    m_encoder.SetPosition(initialPosition);
}

double DragonSparkFlex::GetCounts()
{
    return m_encoder.GetPosition();
}

void DragonSparkFlex::SetRemoteSensor(int canID, ctre::phoenix::motorcontrol::RemoteSensorSource deviceType)
{
}
