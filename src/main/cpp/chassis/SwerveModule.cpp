
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
#include <memory>
#include <numbers>
#include <string>

// FRC includes
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "chassis/SwerveChassis.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "hw/DragonCanCoder.h"
#include "mechanisms/controllers/ControlData.h"
#include "utils/AngleUtils.h"
#include "utils/logging/Logger.h"

// Third Party Includes
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"

using namespace std;
using namespace frc;
using ctre::phoenix6::configs::CANcoderConfiguration;
using ctre::phoenix6::configs::CANcoderConfigurator;
using ctre::phoenix6::configs::MotorOutputConfigs;
using ctre::phoenix6::configs::Slot0Configs;
using ctre::phoenix6::configs::TalonFXConfiguration;
using ctre::phoenix6::controls::DutyCycleOut;
using ctre::phoenix6::controls::PositionVoltage;
using ctre::phoenix6::controls::VelocityTorqueCurrentFOC;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;
using ctre::phoenix6::signals::FeedbackSensorSourceValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix6::signals::SensorDirectionValue;

SwerveModule::SwerveModule(SwerveModuleConstants::ModuleID id,
                           SwerveModuleConstants::ModuleType type,
                           int driveMotorID,
                           bool driveInverted,
                           int turnMotorID,
                           bool turnInverted,
                           int canCoderID,
                           double angleOffset) : LoggableItem(),
                                                 m_moduleID(id),
                                                 m_driveTalon(new TalonFX(driveMotorID, "Canivore")),
                                                 m_turnTalon(new TalonFX(turnMotorID, "Canivore")),
                                                 m_turnCancoder(new CANcoder(canCoderID, "Canivore")),
                                                 m_activeState()
{
    Rotation2d ang{units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;

    auto attrs = SwerveModuleConstants::GetSwerveModuleAttrs(type);
    m_wheelDiameter = attrs.wheelDiameter;
    m_maxSpeed = attrs.maxSpeed;
    m_maxAngSpeed = attrs.maxAngSpeed;

    if (m_driveTalon != nullptr)
    {
        m_driveTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        MotorOutputConfigs motorconfig{};
        motorconfig.Inverted = driveInverted ? InvertedValue::CounterClockwise_Positive : InvertedValue::Clockwise_Positive;
        motorconfig.NeutralMode = NeutralModeValue::Brake;
        motorconfig.PeakForwardDutyCycle = 1.0;
        motorconfig.PeakReverseDutyCycle = -1.0;
        motorconfig.DutyCycleNeutralDeadband = 0.0;
        m_driveTalon->GetConfigurator().Apply(motorconfig);
    }
    if (m_turnTalon != nullptr)
    {
        m_turnTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        MotorOutputConfigs motorconfig{};
        motorconfig.Inverted = driveInverted ? InvertedValue::CounterClockwise_Positive : InvertedValue::Clockwise_Positive;
        motorconfig.NeutralMode = NeutralModeValue::Brake;
        motorconfig.PeakForwardDutyCycle = 1.0;
        motorconfig.PeakReverseDutyCycle = -1.0;
        motorconfig.DutyCycleNeutralDeadband = 0.0;
        m_turnTalon->GetConfigurator().Apply(motorconfig);

        Slot0Configs config{};
        config.kV = m_turnKf;
        config.kP = m_turnKp;
        config.kI = m_turnKi;
        config.kD = m_turnKd;
        m_turnTalon->GetConfigurator().Apply(config, 50_ms);
    }
    if (m_turnCancoder != nullptr)
    {
        m_turnCancoder->GetConfigurator().Apply(CANcoderConfiguration{}); // Apply Factory Defaults

        CANcoderConfiguration configs{};
        configs.MagnetSensor.MagnetOffset = angleOffset;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
        m_turnCancoder->GetConfigurator().Apply(configs);
    }

    if (m_turnTalon != nullptr && m_turnCancoder != nullptr)
    {
        TalonFXConfiguration configs{};
        m_turnTalon->GetConfigurator().Refresh(configs);
        configs.Feedback.FeedbackRemoteSensorID = m_turnCancoder->GetDeviceID();
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::SyncCANcoder;
        configs.Feedback.SensorToMechanismRatio = attrs.sensorToMechanismRatio;
        configs.Feedback.RotorToSensorRatio = attrs.rotorToSensorRatio;
        m_turnTalon->GetConfigurator().Apply(configs);
    }
    LogInformation();
}

/// @brief Set all motor encoders to zero
/// @brief void
void SwerveModule::SetEncodersToZero()
{
    // Add method to set position to 0 and add to drive motor
}

/// @brief Get the encoder values
/// @returns double - the integrated sensor position
double SwerveModule::GetEncoderValues()
{
    return m_driveTalon->GetPosition().GetValueAsDouble() * 2048;
}

/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    // Desired State
    SetTurnAngle(units::degree_t(0));
    LogInformation();
}

/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * numbers::pi);
    units::velocity::meters_per_second_t mps = units::velocity::meters_per_second_t(0.0);
    mps = units::velocity::meters_per_second_t(mpr.to<double>() * m_driveTalon->GetVelocity().GetValueAsDouble());

    // Get the Module Current Rotation Angle
    units::angle::degree_t ang = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d angle = Rotation2d(ang);

    // Create the state and return it
    SwerveModuleState state{mps, angle};
    return state;
}

/// @brief Get the current position of the swerve module (distance and rotation)
/// @return frc::SwerveModulePosition - current position
frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    double rotations = 0.0;
    rotations = m_driveTalon->GetPosition().GetValueAsDouble();
    units::angle::degree_t angle = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);

    return {rotations * m_wheelDiameter * numbers::pi, // distance travled by drive motor
            currAngle};                                // angle of the swerve module from sensor
}

/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState(const SwerveModuleState &targetState)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    units::angle::degree_t angle = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);
    auto optimizedState = SwerveModuleState::Optimize(targetState, currAngle);

    string ntAngleName;
    string ntSpeed;
    if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_BACK)
    {
        ntAngleName += string("leftback target Angle");
        ntSpeed += string("leftback target speed");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_FRONT)
    {
        ntAngleName += string("leftfront target Angle");
        ntSpeed += string("leftfront target speed");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::RIGHT_BACK)
    {
        ntAngleName += string("rightback target Angle");
        ntSpeed += string("rightback target speed");
    }
    else
    {
        ntAngleName += string("rightfront target Angle");
        ntSpeed += string("rightfront target speed");
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), ntAngleName, optimizedState.angle.Degrees().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), ntSpeed, optimizedState.speed.to<double>());

    // Set Turn Target
    SetTurnAngle(optimizedState.angle.Degrees());

    // Set Drive Target
    SetDriveSpeed(optimizedState.speed);

    LogInformation();
}

/// @brief Run the swerve module at the same speed and angle
/// @returns void
void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_activeState.speed);
}

/// @brief run the drive motor at a specified speed
/// @param [in] speed to drive the drive wheel as
/// @returns void
void SwerveModule::SetDriveSpeed(units::velocity::meters_per_second_t speed)
{
    // Run Open Loop
    m_activeState.speed = (abs(speed.to<double>() / m_maxSpeed.to<double>()) < 0.05) ? 0_mps : speed;
    // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
    auto driveTarget = m_activeState.speed.to<double>() / (units::length::meter_t(m_wheelDiameter).to<double>() * numbers::pi);
    auto percent = driveTarget / m_maxSpeed.to<double>();
    DutyCycleOut out{percent};
    m_driveTalon->SetControl(out);
    // VelocityTorqueCurrentFOC out{units::angular_velocity::turns_per_second_t(driveTarget)};
    // out.Slot = 0;
    // m_driveTalon->SetControl(out);
}

/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetTurnAngle(units::angle::degree_t targetAngle)
{
    PositionVoltage voltagePosition{0_tr, 0_tps, true, 0_V, 0, false};
    Slot0Configs config{};
    config.kV = m_turnKf;
    config.kP = m_turnKp;
    config.kI = m_turnKi;
    config.kD = m_turnKd;
    m_turnTalon->GetConfigurator().Apply(config);

    m_activeState.angle = targetAngle;
    m_turnTalon->SetControl(voltagePosition.WithPosition(targetAngle));

    //  m_turnTalon->SetControl(m_voltagePosition.WithPosition(targetAngle));
    //  m_turnTalon->SetControl(m_torquePosition.WithPosition(targetAngle));
}

/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    // TODO: add method to stop motor and do it for both turn and drive motors
}

void SwerveModule::LogInformation()
{
    string ntAngleName;
    string ntMotorPositionName;
    string ntRotorPositionName;
    if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_BACK)
    {
        ntAngleName += string("leftback Angle");
        ntMotorPositionName += string("leftback turns");
        ntRotorPositionName += string("leftback rotor");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_FRONT)
    {
        ntAngleName += string("leftfront Angle");
        ntMotorPositionName += string("leftfront turns");
        ntRotorPositionName += string("leftfront rotor");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::RIGHT_BACK)
    {
        ntAngleName += string("rightback Angle");
        ntMotorPositionName += string("rightback turns");
        ntRotorPositionName += string("rightback rotor");
    }
    else
    {
        ntAngleName += string("rightfront Angle");
        ntMotorPositionName += string("rightfront turns");
        ntRotorPositionName += string("rightfront rotor");
    }
    auto angle = m_turnCancoder->GetAbsolutePosition().GetValue();
    units::angle::degree_t angleDegree = angle;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), ntAngleName, angleDegree.to<double>());

    auto turns = m_turnTalon->GetPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), ntMotorPositionName, turns);

    auto rotor = m_turnTalon->GetRotorPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("swerve"), ntRotorPositionName, turns);
}
