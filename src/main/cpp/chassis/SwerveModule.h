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
#include <memory>
#include <string>

// FRC Includes
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTable.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "utils/logging/LoggableItem.h"

// Team 302 Includes
#include "chassis/SwerveModuleConstants.h"

// Third Party
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

class SwerveModule : public LoggableItem
{
public:
    /// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
    SwerveModule(std::string canbusname,
                 SwerveModuleConstants::ModuleID id,
                 SwerveModuleConstants::ModuleType type,
                 int driveMotorID,
                 bool driveInverted,
                 int turnMotorID,
                 bool turnInverted,
                 int canCoderID,
                 bool canCoderInverted,
                 double angleOffset,
                 std::string configfilename,
                 std::string networkTableName);

    /// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
    /// @returns void
    void ZeroAlignModule();

    ///@brief
    /// @returns
    double GetEncoderValues();

    /// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
    /// @returns SwerveModuleState
    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;

    /// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
    /// @param [in] const SwerveModuleState& referenceState:   state to set the module to
    /// @returns void
    void SetDesiredState(const frc::SwerveModuleState &state);

    void RunCurrentState();

    /// @brief Return which module this is
    /// @returns SwerveModuleConstants.ModuleID
    SwerveModuleConstants::ModuleID GetModuleID() { return m_moduleID; }
    units::length::inch_t GetWheelDiameter() const { return m_wheelDiameter; }
    units::velocity::feet_per_second_t GetMaxSpeed() const { return m_maxSpeed; }
    units::angular_velocity::radians_per_second_t GetRotationalVelocity();

    void StopMotors();
    void LogInformation() override;

private:
    void InitDriveMotor(bool inverted);
    void InitTurnMotorEncoder(
        bool turnInverted,
        bool canCoderInverted,
        double angleOffset,
        const SwerveModuleAttributes &attrs);
    void SetDriveSpeed(units::velocity::meters_per_second_t speed);
    void SetTurnAngle(units::angle::degree_t angle);
    void ReadConstants(std::string configfilename);

    SwerveModuleConstants::ModuleID m_moduleID;
    ctre::phoenix6::hardware::TalonFX *m_driveTalon;
    ctre::phoenix6::hardware::TalonFX *m_turnTalon;
    ctre::phoenix6::hardware::CANcoder *m_turnCancoder;

    frc::SwerveModuleState m_activeState;

    ctre::phoenix6::controls::PositionTorqueCurrentFOC m_torquePosition{0_tr, 0_tps, 0_A, 1, false};
    ctre::phoenix6::controls::PositionVoltage m_voltagePosition{0_tr, 0_tps, true, 0_V, 0, false};

    double m_turnKp = 5.0;
    double m_turnKi = 0.0;
    double m_turnKd = 0.0;
    double m_turnKf = 0.0;
    double m_turnCruiseVel = 0.0;
    double m_turnMaxAcc = 0.0;
    units::length::inch_t m_wheelDiameter = units::length::inch_t(4.0);
    units::velocity::feet_per_second_t m_maxSpeed = units::velocity::feet_per_second_t(16.0);
    std::string m_networkTableName;
};