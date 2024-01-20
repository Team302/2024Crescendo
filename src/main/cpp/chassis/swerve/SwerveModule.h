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
#include "frc/Encoder.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/acceleration.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Team 302 Includes
#include "chassis/PoseEstimatorEnum.h"
#include "chassis/swerve/SwerveModuleConstants.h"
#include "hw/DragonCanCoder.h"
#include "hw/DragonTalonFX.h"
#include "hw/interfaces/IDragonMotorController.h"
#include "mechanisms/controllers/ControlData.h"

// Third Party Includes

class SwerveModule
{
public:
    /// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
    /// @param [in] SwerveModuleConstants.ModuleID                                                type:           Which Swerve Module is it
    /// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move
    /// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module
    /// @param [in] DragonCanCoder*       		                            canCoder:       Sensor for detecting the angle of the wheel
    SwerveModule(SwerveModuleConstants::ModuleID id,
                 SwerveModuleConstants::ModuleType type,
                 IDragonMotorController *driveMotor,
                 IDragonMotorController *turningMotor,
                 DragonCanCoder *canCoder);

    /// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
    /// @returns void
    void ZeroAlignModule();

    /// @brief Set all motor encoders to zero
    /// @returns void
    void SetEncodersToZero();

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

    void StopMotors();

private:
    // Note:  the following was taken from the WPI code and tweaked because we were seeing some weird
    //        reversals that we believe was due to not using a tolerance
    frc::SwerveModuleState Optimize(const frc::SwerveModuleState &desiredState,
                                    const frc::Rotation2d &currentAngle);

    void SetDriveSpeed(units::velocity::meters_per_second_t speed);
    void SetTurnAngle(units::angle::degree_t angle);

    SwerveModuleConstants::ModuleID m_moduleID;

    IDragonMotorController *m_driveMotor;
    IDragonMotorController *m_turnMotor;
    DragonCanCoder *m_turnSensor;

    units::length::inch_t m_wheelDiameter;

    std::string m_nt;

    frc::SwerveModuleState m_activeState;
    frc::Pose2d m_currentPose;
    units::angular_velocity::revolutions_per_minute_t m_currentSpeed;
    double m_currentRotations;

    units::velocity::meters_per_second_t m_maxVelocity;
    bool m_runClosedLoopDrive = true;
    double m_countsOnTurnEncoderPerDegreesOnAngleSensor;
};