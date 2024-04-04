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

// Team 302 includes
#include "chassis/driveStates/FieldDrive.h"
#include "chassis/driveStates/HoldDrive.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/driveStates/StopDrive.h"
#include "chassis/driveStates/StageDrive.h"
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
#include "utils/FMSData.h"
#include "utils/logging/Logger.h"
#include "chassis/driveStates/DriveToNote.h"
#include "utils/FMSData.h"

// Third Party Includes
#include "pugixml/pugixml.hpp"

constexpr int LEFT_FRONT = 0;
constexpr int RIGHT_FRONT = 1;
constexpr int LEFT_BACK = 2;
constexpr int RIGHT_BACK = 3;

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
                                                        m_flState(),
                                                        m_frState(),
                                                        m_blState(),
                                                        m_brState(),
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
    m_drive = moveInfo.chassisSpeeds.vx;
    m_steer = moveInfo.chassisSpeeds.vy;
    m_rotate = moveInfo.chassisSpeeds.omega;

    auto isRotating = (abs(moveInfo.rawOmega) > 0.1);
    if (!isRotating)
    {
        if (m_isRotating)
        {
            m_isRotating = false;
            SetStoredHeading(GetYaw());
        }
    }
    else
    {
        m_isRotating = true;
    }

    m_currentOrientationState = GetHeadingState(moveInfo);
    if (m_currentOrientationState != nullptr)
    {
        m_currentOrientationState->UpdateChassisSpeeds(moveInfo);
    }

    m_currentDriveState = GetDriveState(moveInfo);
    if (m_currentDriveState != nullptr)
    {
        auto states = m_currentDriveState->UpdateSwerveModuleStates(moveInfo);

        m_frontLeft->SetDesiredState(states[LEFT_FRONT]);
        m_frontRight->SetDesiredState(states[RIGHT_FRONT]);
        m_backLeft->SetDesiredState(states[LEFT_BACK]);
        m_backRight->SetDesiredState(states[RIGHT_BACK]);
    }

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
    return m_pigeon->GetYaw().GetValue();
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
    Rotation2d rot2d{m_pigeon->GetYaw().GetValue()};

    m_poseEstimator.Update(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(),
                                                                           m_frontRight->GetPosition(),
                                                                           m_backLeft->GetPosition(),
                                                                           m_backRight->GetPosition()});
    if (m_vision != nullptr)
    {
        std::optional<VisionPose> visionPose = m_vision->GetRobotPosition();
        if (visionPose)
        {

            // only updated based on vision if std deviations are met and difference is under thresholds
            frc::Pose2d chassisPose2d = GetPose();
            frc::Pose2d visionPose2d = visionPose.value().estimatedPose.ToPose2d();
            wpi::array<double, 3> visionMeasurementStdDevs = visionPose.value().visionMeasurementStdDevs;
            units::length::meter_t poseDifference = chassisPose2d.Translation().Distance(visionPose2d.Translation());

            if ((visionMeasurementStdDevs[0] == 0.5) || (poseDifference < units::length::meter_t(0.5) && visionMeasurementStdDevs[0] == 1.0) || (poseDifference < units::length::meter_t(0.3) && visionMeasurementStdDevs[0] == 2.0) && !frc::DriverStation::IsTeleopEnabled())
            {

                m_poseEstimator.AddVisionMeasurement(visionPose.value().estimatedPose.ToPose2d(),
                                                     visionPose.value().timeStamp,
                                                     visionPose.value().visionMeasurementStdDevs);
            }
        }
    }
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
void SwerveChassis::ResetPose(const Pose2d &pose)
{
    ZeroAlignSwerveModules();
    Rotation2d rot2d{GetYaw()};

    m_poseEstimator.ResetPosition(rot2d, wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft->GetPosition(), m_frontRight->GetPosition(), m_backLeft->GetPosition(), m_backRight->GetPosition()}, pose);
}
//=================================================================================
void SwerveChassis::SetYaw(units::angle::degree_t newYaw)
{
    m_pigeon->SetYaw(newYaw);
}

//==================================================================================
void SwerveChassis::ResetYaw()
{
    m_pigeon->Reset();
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
                    else if (strcmp(attr.name(), "maxspeed") == 0)
                    {
                        m_maxSpeed = units::velocity::feet_per_second_t(attr.as_double() / 12.0);
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
