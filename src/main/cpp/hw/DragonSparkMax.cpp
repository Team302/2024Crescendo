
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
                               rev::SparkLimitSwitch::Type forwardType,
                               rev::SparkLimitSwitch::Type reverseType,
                               const DistanceAngleCalcStruc &calcStruc) : IDragonMotorController(),
                                                                          m_id(id),
                                                                          m_spark(new CANSparkMax(id, motorType)),
                                                                          m_outputRotationOffset(0.0),
                                                                          m_deviceType(deviceType),
                                                                          m_feedbackType(feedbackType),
                                                                          m_forwardType(forwardType),
                                                                          m_reverseType(reverseType),
                                                                          m_encoder(m_spark->GetEncoder()),
                                                                          m_pidController(m_spark->GetPIDController()),
                                                                          m_forwardLimitSwitch(m_spark->GetForwardLimitSwitch(m_forwardType)),
                                                                          m_reverseLimitSwitch(m_spark->GetReverseLimitSwitch(m_reverseType)),
                                                                          m_calcStruc(calcStruc)
{
    // m_spark->RestoreFactoryDefaults();
    // // m_spark->SetCANTimeout(0);
    m_pidController.SetOutputRange(-1.0, 1.0, 0);
    m_pidController.SetOutputRange(-1.0, 1.0, 1);
    // m_spark->SetOpenLoopRampRate(0.09); // 0.2 0.25
    // m_spark->SetClosedLoopRampRate(0.02);
    // m_encoder.SetPosition(0);
    // SetRotationOffset(0);
    // m_forwardLimitSwitch.EnableLimitSwitch(false);
    // m_reverseLimitSwitch.EnableLimitSwitch(false);
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
    bool needToSet = false;
    double target = 1.0;
    switch (controlInfo.GetMode())
    {
    case ControlModes::PERCENT_OUTPUT:
        m_controlType = CANSparkBase::ControlType ::kDutyCycle;
        break;
    case ControlModes::POSITION_INCH:
        target = m_calcStruc.countsPerInch;
        needToSet = std::abs(m_posConversion - target) > m_chgTolerance;
        if (needToSet)
        {
            m_encoder.SetPositionConversionFactor(target);
            m_posConversion = target;
        }
        m_controlType = CANSparkBase::ControlType::kPosition;
        break;
    case ControlModes::POSITION_DEGREES:
        target = m_calcStruc.countsPerDegree;
        needToSet = std::abs(m_posConversion - target) > m_chgTolerance;
        if (needToSet)
        {
            m_encoder.SetPositionConversionFactor(target);
            m_posConversion = target;
        }
        m_controlType = CANSparkBase::ControlType ::kPosition;
        break;
    case ControlModes::VELOCITY_RPS:
        target = m_calcStruc.countsPerRev;
        needToSet = std::abs(m_velConversion - target) > m_chgTolerance;
        if (needToSet)
        {
            m_encoder.SetVelocityConversionFactor(target);
            m_velConversion = target;
        }
        m_controlType = CANSparkBase::ControlType::kVelocity;
        break;

    default:
        // danger11!!!!
        m_spark->Set(0);
        break;
    }

    auto ctlSlot = (m_controlType == CANSparkBase::ControlType::kVelocity) ? m_velSlot : m_posSlot;

    target = controlInfo.GetP();
    needToSet = std::abs(m_prevKp[ctlSlot] - target) > m_chgTolerance;
    if (needToSet)
    {
        m_prevKp[ctlSlot] = target;
        m_pidController.SetP(target, ctlSlot);
    }

    target = controlInfo.GetI();
    needToSet = std::abs(m_prevKi[ctlSlot] - target) > m_chgTolerance;
    if (needToSet)
    {
        m_prevKi[ctlSlot] = target;
        m_pidController.SetI(target, ctlSlot);
    }

    target = controlInfo.GetD();
    needToSet = std::abs(m_prevKd[ctlSlot] - target) > m_chgTolerance;
    if (needToSet)
    {
        m_prevKd[ctlSlot] = target;
        m_pidController.SetD(target, ctlSlot);
    }

    target = controlInfo.GetF();
    needToSet = std::abs(m_prevKf[ctlSlot] - target) > m_chgTolerance;
    if (needToSet)
    {
        m_prevKf[ctlSlot] = target;
        m_pidController.SetFF(target, ctlSlot);
    }
}

void DragonSparkMax::EnableCurrentLimiting(bool enabled)
{
}

void DragonSparkMax::Set(double value)
{
    if (m_controlType == CANSparkBase::ControlType::kDutyCycle)
    {
        m_spark->Set(value);
    }
    else if (m_controlType == CANSparkBase::ControlType::kVelocity)
    {
        m_pidController.SetReference(value * 60, m_controlType, m_velSlot);
    }
    else
    {
        m_pidController.SetReference(value, m_controlType, m_posSlot);
    }
}

void DragonSparkMax::SetRotationOffset(double rotations)
{
    // m_outputRotationOffset = GetRotationsWithGearNoOffset() - rotations;
}

void DragonSparkMax::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
    // m_spark->SetOpenLoopRampRate(ramping);
    // m_spark->SetClosedLoopRampRate(ramping); // TODO: should closed and open be separate

    // if (rampingClosedLoop >= 0)
    // {
    //     m_spark->SetClosedLoopRampRate(rampingClosedLoop);
    // }
}

void DragonSparkMax::EnableBrakeMode(bool enabled)
{
    // m_spark->SetIdleMode(enabled ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
}

void DragonSparkMax::Invert(bool inverted)
{
    m_spark->SetInverted(inverted);
}

double DragonSparkMax::GetRotationsWithGearNoOffset() const
{
    return m_encoder.GetPosition() * m_calcStruc.gearRatio;
}

void DragonSparkMax::InvertEncoder(bool inverted)
{
    // m_encoder.SetInverted(inverted);
}

CANSparkMax *DragonSparkMax::GetSparkMax()
{
    return m_spark;
}

void DragonSparkMax::SetSmartCurrentLimiting(int limit)
{
    // m_spark->SetSmartCurrentLimit(limit);
}

void DragonSparkMax::SetSecondaryCurrentLimiting(int limit, int duration)
{
    // m_spark->SetSecondaryCurrentLimit(limit, duration);
}

double DragonSparkMax::GetCurrent()
{
    return m_spark->GetOutputCurrent();
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
    return m_spark->GetInverted();
}

bool DragonSparkMax::IsForwardLimitSwitchClosed()
{
    return m_forwardLimitSwitch.Get();
}

bool DragonSparkMax::IsReverseLimitSwitchClosed()
{
    return m_reverseLimitSwitch.Get();
}

void DragonSparkMax::EnableDisableLimitSwitches(bool enable)
{
    m_forwardLimitSwitch.EnableLimitSwitch(enable);
    m_reverseLimitSwitch.EnableLimitSwitch(enable);
}

void DragonSparkMax::EnableVoltageCompensation(double fullvoltage)
{
    m_spark->EnableVoltageCompensation(fullvoltage);
}

void DragonSparkMax::SetSelectedSensorPosition(
    double initialPosition)
{
    m_encoder.SetPosition(initialPosition);
}

double DragonSparkMax::GetCounts()
{
    return m_encoder.GetPosition();
}

void DragonSparkMax::SetRemoteSensor(int canID, ctre::phoenix::motorcontrol::RemoteSensorSource deviceType)
{
}

void DragonSparkMax::MonitorCurrent()
{
}