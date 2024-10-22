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
#include <numbers>
#include <cmath>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Rotation2d.h"
#include "units/angular_acceleration.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/DataLogManager.h"
// #include "wpi/DataLog.h"

// Team 302 includes
#include "chassis/driveStates/DriveToNote.h"
#include "chassis/driveStates/FieldDrive.h"
#include "chassis/driveStates/HoldDrive.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/driveStates/StageDrive.h"
#include "chassis/driveStates/StopDrive.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "chassis/headingStates/FaceAmp.h"
#include "chassis/headingStates/FaceCenterStage.h"
#include "chassis/headingStates/FaceGamePiece.h"
#include "chassis/headingStates/FaceLeftStage.h"
#include "chassis/headingStates/FaceRightStage.h"
#include "chassis/headingStates/FaceSpeaker.h"
#include "chassis/headingStates/FaceStage.h"
#include "chassis/headingStates/IgnoreHeading.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "chassis/headingStates/MaintainHeading.h"
#include "chassis/headingStates/SpecifiedHeading.h"
#include "chassis/LogChassisMovement.h"
#include "chassis/SwerveChassis.h"
#include "utils/logging/Logger.h"
#include "utils/AngleUtils.h"

// Third Party Includes
#include "pugixml/pugixml.hpp"

using std::map;
using std::string;

using frc::ChassisSpeeds;
using frc::Pose2d;
using frc::Rotation2d;
using frc::SwerveModulePosition;

using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;

/// @brief Construct a swerve chassis
SwerveChassis::SwerveChassis(SwerveModule *frontLeft,
                             SwerveModule *frontRight,
                             SwerveModule *backLeft,
                             SwerveModule *backRight,
                             Pigeon2 *pigeon,
                             string configfilename,
                             string networkTableName) : IChassis(),
                                                        LoggableItem(),
                                                        m_frontLeft(frontLeft),
                                                        m_frontRight(frontRight),
                                                        m_backLeft(backLeft),
                                                        m_backRight(backRight),
                                                        m_pigeon(pigeon),
                                                        m_robotDrive(nullptr),
                                                        m_frontLeftLocation(units::length::inch_t(22.5 / 2.0), units::length::inch_t(22.5 / 2.0)),
                                                        m_frontRightLocation(units::length::inch_t(22.5 / 2.0), units::length::inch_t(-22.5 / 2.0)),
                                                        m_backLeftLocation(units::length::inch_t(-22.5 / 2.0), units::length::inch_t(22.5 / 2.0)),
                                                        m_backRightLocation(units::length::inch_t(-22.5 / 2.0), units::length::inch_t(-22.5 / 2.0)),
                                                        m_kinematics(m_frontLeftLocation,
                                                                     m_frontRightLocation,
                                                                     m_backLeftLocation,
                                                                     m_backRightLocation),
                                                        m_poseEstimator(m_kinematics,
                                                                        frc::Rotation2d{},
                                                                        {SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition()},
                                                                        frc::Pose2d(),
                                                                        {0.1, 0.1, 0.1},
                                                                        {0.1, 0.1, 0.1}),
                                                        m_storedYaw(units::angle::degree_t(0.0)),
                                                        m_targetHeading(units::angle::degree_t(0.0)),
                                                        m_networkTableName(networkTableName),
                                                        m_vision(DragonVision::GetDragonVision())
{
    ReadConstants(configfilename);
    InitStates();
    ZeroAlignSwerveModules();
    ResetYaw();
    ResetPose(frc::Pose2d());
    SetStoredHeading(units::angle::degree_t(0.0));
    m_maxSpeed = m_frontLeft->GetMaxSpeed();
    m_velocityTimer.Reset();
    m_radius = m_frontLeftLocation.Norm();
}

//==================================================================================
void SwerveChassis::InitStates()
{
    m_robotDrive = new RobotDrive(this);
    auto trajectoryDrivePathPlanner = new TrajectoryDrivePathPlanner(m_robotDrive);

    m_driveStateMap[ChassisOptionEnums::DriveStateType::FIELD_DRIVE] = new FieldDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::HOLD_DRIVE] = new HoldDrive();
    m_driveStateMap[ChassisOptionEnums::DriveStateType::STAGE_DRIVE] = new StageDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::ROBOT_DRIVE] = m_robotDrive;
    m_driveStateMap[ChassisOptionEnums::DriveStateType::STOP_DRIVE] = new StopDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER] = new TrajectoryDrivePathPlanner(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE] = new DriveToNote(m_robotDrive, trajectoryDrivePathPlanner);

    m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN] = new MaintainHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE] = new SpecifiedHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE] = new FaceGamePiece();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::IGNORE] = new IgnoreHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_AMP] = new FaceAmp();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_SPEAKER] = new FaceSpeaker();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_CENTER_STAGE] = new FaceCenterStage();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_LEFT_STAGE] = new FaceLeftStage();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_RIGHT_STAGE] = new FaceRightStage();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_STAGE] = new FaceStage();
}

//==================================================================================
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();
}

//==================================================================================
/// @brief Drive the chassis
void SwerveChassis::Drive(ChassisMovement &moveInfo)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassisYaw", string("Yaw"), GetYaw().value());

    m_drive = moveInfo.chassisSpeeds.vx;
    m_steer = moveInfo.chassisSpeeds.vy;
    m_rotate = moveInfo.chassisSpeeds.omega;

    if (abs(moveInfo.rawOmega) > 0.05)
    {
        m_rotatingLatch = true;
    }
    else if (abs(GetRotationRateDegreesPerSecond()) < 5.0) // degrees per second
    {
        m_rotatingLatch = false;
    }

    if (m_rotatingLatch)
    {
        SetStoredHeading(GetYaw());
    }

    m_currentOrientationState = GetHeadingState(moveInfo);
    if (m_currentOrientationState != nullptr)
    {
        m_currentOrientationState->UpdateChassisSpeeds(moveInfo);
    }

    m_currentDriveState = GetDriveState(moveInfo);
    if (m_currentDriveState != nullptr)
    {
        m_targetStates = m_currentDriveState->UpdateSwerveModuleStates(moveInfo);

        m_frontLeft->SetDesiredState(m_targetStates[LEFT_FRONT], GetInertialVelocity(), units::degrees_per_second_t(GetRotationRateDegreesPerSecond()), m_radius);
        m_frontRight->SetDesiredState(m_targetStates[RIGHT_FRONT], GetInertialVelocity(), units::degrees_per_second_t(GetRotationRateDegreesPerSecond()), m_radius);
        m_backLeft->SetDesiredState(m_targetStates[LEFT_BACK], GetInertialVelocity(), units::degrees_per_second_t(GetRotationRateDegreesPerSecond()), m_radius);
        m_backRight->SetDesiredState(m_targetStates[RIGHT_BACK], GetInertialVelocity(), units::degrees_per_second_t(GetRotationRateDegreesPerSecond()), m_radius);
    }
    m_rotate = moveInfo.chassisSpeeds.omega;
    UpdateOdometry();
}

//==================================================================================
ISwerveDriveState *SwerveChassis::GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType driveOption)
{
    auto itr = m_driveStateMap.find(driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveOrientation *SwerveChassis::GetSpecifiedHeadingState(ChassisOptionEnums::HeadingOption headingOption)
{
    auto itr = m_headingStateMap.find(headingOption);
    if (itr == m_headingStateMap.end())
    {
        return m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN];
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveOrientation *SwerveChassis::GetHeadingState(const ChassisMovement &moveInfo)
{
    auto itr = m_headingStateMap.find(moveInfo.headingOption);
    if (itr == m_headingStateMap.end())
    {
        itr = m_headingStateMap.find(ChassisOptionEnums::HeadingOption::MAINTAIN);
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveState *SwerveChassis::GetDriveState(ChassisMovement &moveInfo)
{
    auto state = GetSpecifiedDriveState(moveInfo.driveOption);

    auto itr = m_driveStateMap.find(moveInfo.driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    state = itr->second;

    if (m_currentDriveState == nullptr)
    {
        m_currentDriveState = m_robotDrive;
    }

    if (state != m_currentDriveState)
    {
        m_initialized = false;
    }

    if (!m_initialized && state != nullptr)
    {
        state->Init(moveInfo);
        m_initialized = true;
    }

    return state;
}

//==================================================================================
Pose2d SwerveChassis::GetPose() const
{
    return m_poseEstimator.GetEstimatedPosition();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetYaw() const
{
    return m_pigeon->GetYaw().WaitForUpdate(100_ms).Refresh().GetValue();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetPitch() const
{
    return m_pigeon->GetPitch().GetValue();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetRoll() const
{
    return m_pigeon->GetRoll().GetValue();
}

//==================================================================================
/// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
void SwerveChassis::UpdateOdometry()
{

    bool updateWithVision = false;
    Rotation2d rot2d{GetYaw()};

    m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(),
                                                                           m_frontRight->GetPosition(),
                                                                           m_backLeft->GetPosition(),
                                                                           m_backRight->GetPosition()});
    if (m_vision != nullptr)
    {
        auto useVision = (m_pigeon != nullptr && std::abs(GetRotationRateDegreesPerSecond()) < 720.0);
        if (useVision)
        {
            auto hasValidRotation = true;
            auto rotation = GetYaw();
            auto hadBrownOut = m_pigeon->GetStickyFault_Undervoltage().GetValue();
            if (hadBrownOut)
            {
                hasValidRotation = false;

                auto visionPosition = m_vision->GetRobotPosition();
                auto hasVisionPose = visionPosition.has_value();
                if (hasVisionPose)
                {
                    rotation = visionPosition.value().estimatedPose.ToPose2d().Rotation().Degrees();
                    m_pigeon->ClearStickyFault_Undervoltage();
                    hasValidRotation = true;
                    SetYaw(rotation);
                    SetStoredHeading(rotation);
                }
            }
            if (hasValidRotation)
            {
                std::optional<VisionPose> megaTag2Pose = m_vision->GetRobotPositionMegaTag2(rotation,
                                                                                            units::angular_velocity::degrees_per_second_t(0.0),
                                                                                            units::angle::degree_t(0.0),
                                                                                            units::angular_velocity::degrees_per_second_t(0.0),
                                                                                            units::angle::degree_t(0.0),
                                                                                            units::angular_velocity::degrees_per_second_t(0.0));

                if (megaTag2Pose)
                {
                    if (hadBrownOut)
                    {
                        ResetPose(megaTag2Pose.value().estimatedPose.ToPose2d());
                    }
                    else
                    {
                        m_poseEstimator.SetVisionMeasurementStdDevs(megaTag2Pose->visionMeasurementStdDevs); // wpi::array<double, 3>(.7, .7, 9999999));

                        m_poseEstimator.AddVisionMeasurement(megaTag2Pose.value().estimatedPose.ToPose2d(),
                                                             megaTag2Pose.value().timeStamp);
                    }
                    updateWithVision = true;
                }
            }
        }
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Update With Vision"), std::string("Update With Vision:"), updateWithVision);
    LogInformation();
}

//==================================================================================
double SwerveChassis::GetEncoderValues(SwerveModule *motor)
{
    return motor->GetEncoderValues();
}

//==================================================================================
/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({m_frontLeft->GetState(),
                                         m_frontRight->GetState(),
                                         m_backLeft->GetState(),
                                         m_backRight->GetState()});
}
//==================================================================================

void SwerveChassis::LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES swerveModule)
{
    if (swerveModule == SwerveChassis::SWERVE_MODULES::RIGHT_FRONT)
    {
        m_frontRight->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::RIGHT_BACK)
    {
        m_backRight->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::LEFT_FRONT)
    {
        m_frontLeft->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::LEFT_BACK)
    {
        m_backLeft->LogInformation();
    }
}

//==================================================================================
void SwerveChassis::ResetPose(const Pose2d &pose)
{
    ZeroAlignSwerveModules();
    Rotation2d rot2d{pose.Rotation().Degrees()};
    SetStoredHeading(pose.Rotation().Degrees());

    m_poseEstimator.ResetPosition(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(), m_frontRight->GetPosition(), m_backLeft->GetPosition(), m_backRight->GetPosition()}, pose);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassisLogging", string("ResetPosePigeonYaw"), m_pigeon->GetYaw().GetValue().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassisLogging", string("ResetPoseEstimatedYaw"), m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value());
}
//=================================================================================
void SwerveChassis::SetYaw(units::angle::degree_t newYaw)
{
    auto status = m_pigeon->SetYaw(newYaw, units::time::second_t(0.1));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassis::SetYaw", string("status"), status.GetName());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassis::SetYaw", string("status error"), status.IsError() ? "true" : "false");
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "SwerveChassis::SetYaw", string("status ok"), status.IsOK() ? "true" : "false");
}

//==================================================================================
void SwerveChassis::ResetYaw()
{
    m_pigeon->Reset();
    SetStoredHeading(units::angle::degree_t(0));
    ZeroAlignSwerveModules();
}

//==================================================================================
void SwerveChassis::SetStoredHeading(units::angle::degree_t heading)
{
    m_storedYaw = heading;
}

//==================================================================================
void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw)
{
    m_targetHeading = targetYaw;
}

//==================================================================================
units::length::inch_t SwerveChassis::GetWheelDiameter() const
{
    return m_wheelDiameter;
}

//==================================================================================
units::velocity::meters_per_second_t SwerveChassis::GetMaxSpeed() const
{
    return m_maxSpeed;
}

//==================================================================================
units::angular_velocity::radians_per_second_t SwerveChassis::GetMaxAngularSpeed() const
{
    units::length::meter_t circumference = std::numbers::pi * m_wheelBase * .707 * 2.0;
    auto angSpeed = units::angular_velocity::turns_per_second_t(GetMaxSpeed().to<double>() / circumference.to<double>());
    units::angular_velocity::radians_per_second_t retval = angSpeed;
    return retval;
}

//==================================================================================
units::velocity::meters_per_second_t SwerveChassis::GetInertialVelocity()
{
    units::acceleration::meters_per_second_squared_t accelerationX = m_pigeon->GetAccelerationX().GetValue();
    units::acceleration::meters_per_second_squared_t accelerationY = m_pigeon->GetAccelerationY().GetValue();

    units::time::second_t deltaTime = m_velocityTimer.Get();

    m_velocityTimer.Reset();
    m_velocityTimer.Start();

    units::velocity::meters_per_second_t velocityX = accelerationX * deltaTime;
    units::velocity::meters_per_second_t velocityY = accelerationY * deltaTime;

    return units::velocity::meters_per_second_t(std::sqrt(std::pow(velocityX.to<double>(), 2) + std::pow(velocityY.to<double>(), 2)));
}

//==================================================================================
void SwerveChassis::LogInformation()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Vx"), m_drive.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Vy"), m_steer.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Omega"), m_rotate.to<double>());
    auto pose = GetPose();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current x position"), pose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current y position"), pose.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current rotation position"), pose.Rotation().Degrees().to<double>());
}

void SwerveChassis::DataLog()
{

    LogPoseData(DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_POSE, GetPose());

    LogDoubleData(DragonDataLoggerSignals::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES, GetStoredHeading().value());
    LogDoubleData(DragonDataLoggerSignals::DoubleSignals::CHASSIS_YAW_DEGREES, AngleUtils::GetEquivAngle(GetYaw()).value());

    frc::ChassisSpeeds targetSpeed;
    targetSpeed.vx = m_drive;
    targetSpeed.vy = m_steer;
    targetSpeed.omega = m_rotate;
    LogChassisSpeedsData(DragonDataLoggerSignals::ChassisSpeedSignals::TARGET_SPEEDS, targetSpeed);

    auto currFrontLeftState = m_frontLeft->GetState();
    auto currFrontRightState = m_frontRight->GetState();
    auto currBackLeftState = m_backLeft->GetState();
    auto currBackRightState = m_backRight->GetState();
    wpi::array<frc::SwerveModuleState, 4> states = {currFrontLeftState, currFrontRightState, currBackLeftState, currBackRightState};
    auto currentSpeed = m_kinematics.ToChassisSpeeds(states);
    LogChassisSpeedsData(DragonDataLoggerSignals::ChassisSpeedSignals::ACTUAL_SPEEDS, currentSpeed);

    auto optFrontLeftState = m_frontLeft->GetOptimizedState();
    auto optFrontRightState = m_frontRight->GetOptimizedState();
    auto optBackLeftState = m_backLeft->GetOptimizedState();
    auto optBackRightState = m_backRight->GetOptimizedState();

    auto frontLeftSlip = m_frontLeft->IsSlipping();
    auto frontRightSlip = m_frontRight->IsSlipping();
    auto backLeftSlip = m_backLeft->IsSlipping();
    auto backRightSlip = m_backRight->IsSlipping();

    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_FRONT_STATE, optFrontLeftState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_BACK_STATE, optBackLeftState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE, optFrontRightState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_BACK_STATE, optBackRightState);

    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE, currFrontLeftState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE, currBackLeftState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE, currFrontRightState);
    LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE, currBackRightState);

    LogBoolData(DragonDataLoggerSignals::BoolSignals::FRONT_LEFT_SWERVE_MODULE_SPLIPING, frontLeftSlip);
    LogBoolData(DragonDataLoggerSignals::BoolSignals::FRONT_RIGHT_SWERVE_MODULE_SPLIPING, frontRightSlip);
    LogBoolData(DragonDataLoggerSignals::BoolSignals::BACK_LEFT_SWERVE_MODULE_SPLIPING, backLeftSlip);
    LogBoolData(DragonDataLoggerSignals::BoolSignals::BACK_RIGHT_SWERVE_MODULE_SPLIPING, backRightSlip);

    if (m_currentDriveState != nullptr)
    {
        LogStringData(DragonDataLoggerSignals::StringSignals::CHASSIS_DRIVE_STATE, m_currentDriveState->GetDriveStateName());
    }
    if (m_currentOrientationState != nullptr)
    {
        LogStringData(DragonDataLoggerSignals::StringSignals::CHASSIS_HEADING_STATE, m_currentOrientationState->GetHeadingStateName());
    }
}

//==================================================================================
void SwerveChassis::ReadConstants(string configfilename)
{
    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto filename = deployDir + string("/chassis/") + configfilename;
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());

    if (result)
    {
        pugi::xml_node parent = doc.root();
        for (pugi::xml_node swervemod = parent.first_child(); swervemod; swervemod = swervemod.next_sibling())
        {
            for (pugi::xml_node control = swervemod.first_child(); swervemod; swervemod = swervemod.next_sibling())
            {
                for (pugi::xml_attribute attr = control.first_attribute(); attr; attr = attr.next_attribute())
                {
                    if (strcmp(attr.name(), "wheelbase") == 0)
                    {
                        m_wheelBase = units::length::inch_t(attr.as_double());
                    }
                    else if (strcmp(attr.name(), "track") == 0)
                    {
                        m_track = units::length::inch_t(attr.as_double());
                    }
                    else if (strcmp(attr.name(), "wheeldiameter") == 0)
                    {
                        m_wheelDiameter = units::length::inch_t(attr.as_double());
                    }
                }
            }
        }
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, m_networkTableName, string("Config File not found"), configfilename);
    }
}
