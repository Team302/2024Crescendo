
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
#include <map>
#include <memory>
#include <string>

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModuleState.h"

#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

#include "chassis/ChassisOptionEnums.h"
#include "chassis/driveStates/ISwerveDriveState.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "chassis/IChassis.h"
#include "chassis/SwerveModule.h"
#include "chassis/ChassisMovement.h"
#include "utils/logging/LoggableItem.h"
#include "chassis/SwerveChassis.h"

#include "ctre/phoenix6/Pigeon2.hpp"

using std::string;

class RobotDrive;

class DragonSwervePoseEstimator
{
public:
    /// @brief Construct a swerve chassis
    /// @param [in] SwerveModule*           frontleft:          front left swerve module
    /// @param [in] SwerveModule*           frontright:         front right swerve module
    /// @param [in] SwerveModule*           backleft:           back left swerve module
    /// @param [in] SwerveModule*           backright:          back right swerve module
    /// @param [in] units::length::inch_t                   wheelDiameter:      Diameter of the wheel
    /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
    /// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
    DragonSwervePoseEstimator(SwerveModule *frontLeft,
                              SwerveModule *frontRight,
                              SwerveModule *backLeft,
                              SwerveModule *backRight,

                              ctre::phoenix6::hardware::Pigeon2 *pigeon,
                              frc::SwerveDrivePoseEstimator<4> poseEstimator,
                              string networkTableName);

    ~DragonSwervePoseEstimator() = default;

    /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
    void UpdateOdometry();

    /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
    /// @param [in] const Pose2d&       pose        Current XY position
    void ResetPose(const frc::Pose2d &pose);

    /// @brief Align all of the swerve modules to point forward
    void ZeroAlignSwerveModulesDSPE();
    /// @brief Sets of the motor encoders to zero
    void SetEncodersToZeroDSPE();

    // The DSPE functions are ones that do the same thing as the non-DSPE ones in SwerveChassis but apply to DragonSwervePoseEstimator functions like ResetPose

    frc::Pose2d GetPose() const;

    // units::length::inch_t GetTrack() const { return m_track; }

    frc::SwerveDriveKinematics<4> GetKinematics() const { return m_kinematics; }

private:
    ISwerveDriveState *GetDriveState(ChassisMovement moveInfo);

    SwerveModule *m_frontLeft;
    SwerveModule *m_frontRight;
    SwerveModule *m_backLeft;
    SwerveModule *m_backRight;

    /*  RobotDrive *m_robotDrive;
        std::map<ChassisOptionEnums::DriveStateType, ISwerveDriveState *> m_driveStateMap;
        std::map<ChassisOptionEnums::HeadingOption, ISwerveDriveOrientation *> m_headingStateMap;

        frc::SwerveModuleState m_flState;
        frc::SwerveModuleState m_frState;
        frc::SwerveModuleState m_blState;
        frc::SwerveModuleState m_brState;
    */
    ctre::phoenix6::hardware::Pigeon2 *m_pigeon;
    units::velocity::meters_per_second_t m_drive;
    units::velocity::meters_per_second_t m_steer;
    units::angular_velocity::radians_per_second_t m_rotate;

    frc::Translation2d m_frontLeftLocation;
    frc::Translation2d m_frontRightLocation;
    frc::Translation2d m_backLeftLocation;
    frc::Translation2d m_backRightLocation;

    frc::SwerveDriveKinematics<4> m_kinematics;

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    units::angle::degree_t m_storedYaw;

    // units::length::inch_t m_track;
    //  units::angle::degree_t m_targetHeading;

    ISwerveDriveState *m_currentDriveState;
    ISwerveDriveOrientation *m_currentOrientationState;

    bool m_initialized = false;
    std::string m_networkTableName;
    // SwerveChassis *m_chassis;
};