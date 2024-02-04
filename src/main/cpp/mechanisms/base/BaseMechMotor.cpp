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

// C++ Includes
#include <string>

// FRC includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "hw/interfaces/IDragonMotorController.h"
#include "configs/RobotElementNames.h"
#include "mechanisms/base/BaseMechMotor.h"
#include "mechanisms/controllers/ControlData.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;

/// @brief A motor that can be added to a mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] IDragonMotorController& motor controller used by this mechanism
/// @param [in] EndOfTravelSensorOption minimum end of travel sensor option
/// @param [in] DragonDigitalInput* minimium end of travel sensor if plugged into RoboRio otherwise ignored
/// @param [in] EndOfTravelSensorOption maximum end of travel sensor option
/// @param [in] DragonDigitalInput* maximum end of travel sensor if plugged into RoboRio otherwise ignored
BaseMechMotor::BaseMechMotor(std::string networkTableName,
                             IDragonMotorController *motorController,
                             EndOfTravelSensorOption minEndOfTravelOption,
                             DragonDigitalInput *minSensor,
                             EndOfTravelSensorOption maxEndOfTravelOption,
                             DragonDigitalInput *maxSensor) : IBaseMechMotor(),
                                                              LoggableItem(),
                                                              m_networkTableName(networkTableName),
                                                              m_motor(motorController),
                                                              m_minEndOfTravelOption(minEndOfTravelOption),
                                                              m_minRoboRioDigital(minSensor),
                                                              m_maxEndOfTravelOption(maxEndOfTravelOption),
                                                              m_maxRoboRioDigital(maxSensor),
                                                              m_targetType(MotorTargetType::PERCENT_OUTPUT),
                                                              m_target(0.0),
                                                              m_targetAngle(units::angle::degree_t(0.0)),
                                                              m_targetAngularVelocity(units::angular_velocity::degrees_per_second_t(0.0)),
                                                              m_targetPosition(units::length::inch_t(0.0)),
                                                              m_targetVelocity(units::velocity::feet_per_second_t(0.0))
{
}

void BaseMechMotor::Update()
{
    m_motor->Set(m_target);
    LogInformation();
}

void BaseMechMotor::SetTargetControl(double percentOutput)
{
    UpdateTarget(percentOutput);
}

void BaseMechMotor::SetTargetControl(ControlData *controlConst, units::angle::degree_t angle)
{
    m_controlData = controlConst;
    SetControlConstants(0, *m_controlData); // todo pass by pointer so as not to make a copy
    UpdateTarget(angle);
}

void BaseMechMotor::SetTargetControl(ControlData *controlConst, units::angular_velocity::revolutions_per_minute_t angVel)
{
    m_controlData = controlConst;
    SetControlConstants(0, *m_controlData);
    UpdateTarget(angVel);
}
void BaseMechMotor::SetTargetControl(ControlData *controlConst, units::length::inch_t position)
{
    m_controlData = controlConst;
    SetControlConstants(0, *m_controlData);
    UpdateTarget(position);
}
void BaseMechMotor::SetTargetControl(ControlData *controlConst, units::velocity::feet_per_second_t velocity)
{
    m_controlData = controlConst;
    SetControlConstants(0, *m_controlData);
    UpdateTarget(velocity);
}

void BaseMechMotor::UpdateTarget(double target)
{
    m_targetType = MotorTargetType::PERCENT_OUTPUT;
    m_target = target;
    Update();
}

void BaseMechMotor::UpdateTarget(units::length::inch_t position)
{
    m_targetType = MotorTargetType::POSITION;
    m_targetPosition = position;
    SetTarget(position.to<double>());
    Update();
}
void BaseMechMotor::UpdateTarget(units::velocity::feet_per_second_t fps)
{
    m_targetType = MotorTargetType::VELOCITY;
    m_targetVelocity = fps;
    SetTarget(fps.to<double>() * 12.0);
    Update();
}

void BaseMechMotor::UpdateTarget(units::angle::degree_t angle)
{
    m_targetType = MotorTargetType::ANGLE;
    m_targetAngle = angle;
    SetTarget(angle.to<double>());
    Update();
}
void BaseMechMotor::UpdateTarget(units::angular_velocity::revolutions_per_minute_t rpm)
{
    m_targetType = MotorTargetType::ANGULAR_VELOCITY;
    m_targetAngularVelocity = rpm;
    SetTarget(rpm.to<double>() / 60.0);
    Update();
}

bool BaseMechMotor::AtTarget()
{
    auto pctError = 0.0;
    if (m_targetType == MotorTargetType::PERCENT_OUTPUT)
    {
        pctError = (m_target - GetTarget()) / m_target; // todo this will always be 0
    }
    else if (m_targetType == MotorTargetType::ANGLE)
    {
        pctError = (m_targetAngle - GetPositionDegrees()) / m_targetAngle;
    }
    else if (m_targetType == MotorTargetType::ANGULAR_VELOCITY)
    {
        pctError = (m_targetAngularVelocity - GetRPM()) / m_targetAngularVelocity;
    }
    else if (m_targetType == MotorTargetType::POSITION)
    {
        pctError = (m_targetPosition - GetPositionInches()) / m_targetPosition;
    }
    else if (m_targetType == MotorTargetType::VELOCITY)
    {
        pctError = (m_targetVelocity - GetFeetPerSec()) / m_targetVelocity;
    }
    return pctError < 0.02; // TODO: this might need to be configurable
}

units::length::inch_t BaseMechMotor::GetPositionInches()
{
    return units::length::inch_t(GetMotor()->GetCounts() / GetMotor()->GetCountsPerInch());
}

units::velocity::feet_per_second_t BaseMechMotor::GetFeetPerSec()
{
    return units::velocity::feet_per_second_t(GetMotor()->GetRPS() / GetMotor()->GetCountsPerInch());
}

units::angle::degree_t BaseMechMotor::GetPositionDegrees()
{
    return units::angle::degree_t(GetMotor()->GetCounts() / GetMotor()->GetCountsPerDegree());
}

units::angular_velocity::revolutions_per_minute_t BaseMechMotor::GetRPM()
{
    return units::angular_velocity::revolutions_per_minute_t(GetMotor()->GetRPS() / 60.0);
}

bool BaseMechMotor::IsAtMinTravel() const
{
    if (m_minEndOfTravelOption == EndOfTravelSensorOption::DIO_IN_MOTOR_CONTROLLER)
    {
    }
    else if (m_minEndOfTravelOption == EndOfTravelSensorOption::DIGITAL_INPUT_IN_ROBORIO &&
             m_minRoboRioDigital != nullptr)
    {
        return m_minRoboRioDigital->Get();
    }
    return false;
}

bool BaseMechMotor::IsAtMaxTravel() const
{
    if (m_maxEndOfTravelOption == EndOfTravelSensorOption::DIO_IN_MOTOR_CONTROLLER)
    {
    }
    else if (m_maxEndOfTravelOption == EndOfTravelSensorOption::DIGITAL_INPUT_IN_ROBORIO &&
             m_maxRoboRioDigital != nullptr)
    {
        return m_maxRoboRioDigital->Get();
    }
    return false;
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData* pid:  the control constants
/// @return void
void BaseMechMotor::SetControlConstants(int slot, ControlData pid)
{
    m_motor->SetControlConstants(slot, pid);
}

/// @brief log data to the network table if it is activated and time period has past
void BaseMechMotor::LogInformation()
{
    // todo how to replace this
    // auto usageMap = RobotElementNames::GetInstance();
    // auto usage = usageMap->GetUsage(GetMotor().GetType());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, usage + "Target", GetTarget());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, GetNetworkTableName(), usage + "angular velocity - RPM", GetRPM().to<double>());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, GetNetworkTableName(), usage + "position - degrees", GetPositionDegrees().to<double>());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, GetNetworkTableName(), usage + "velocity - FPS", GetFeetPerSec().to<double>());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, GetNetworkTableName(), usage + "position - inch", GetPositionInches().to<double>());
}
