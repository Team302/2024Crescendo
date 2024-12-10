
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

#include "utils/logging/DragonDataLoggerSignals.h"
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"

DragonDataLoggerSignals *DragonDataLoggerSignals::m_instance = nullptr;
DragonDataLoggerSignals *DragonDataLoggerSignals::GetInstance()
{
    if (DragonDataLoggerSignals::m_instance == nullptr)
    {
        DragonDataLoggerSignals::m_instance = new DragonDataLoggerSignals();
    }
    return DragonDataLoggerSignals::m_instance;
}

DragonDataLoggerSignals::DragonDataLoggerSignals()
{
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    m_hasVision = wpi::log::BooleanLogEntry(log, "/NoteManager/HasVision");
    m_hasVision.Append(m_currHasVision);

    m_frontSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/FrontSensor");
    m_frontSensor.Append(m_currFrontSensor);

    m_backSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/BackSensor");
    m_backSensor.Append(m_currBackSensor);

    m_feederSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/FeederSensor");
    m_feederSensor.Append(m_currFeederSensor);

    m_launcherSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/LauncherSensor");
    m_launcherSensor.Append(m_currLauncherSensor);

    m_placerInSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/PlacerInSensor");
    m_placerInSensor.Append(m_currPlacerInSensor);

    m_placerMidSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/PlacerMidSensor");
    m_placerMidSensor.Append(m_currPlacerMidSensor);

    m_placerOutSensor = wpi::log::BooleanLogEntry(log, "/NoteManager/PlacerOutSensor");
    m_placerOutSensor.Append(m_currPlacerOutSensor);

    m_storedHeading = wpi::log::DoubleLogEntry(log, "/Chassis/StoredHeading(Degrees)");
    m_storedHeading.Append(m_currStoredHeading);
    m_chassisYaw = wpi::log::DoubleLogEntry(log, "/Chassis/Yaw(Degrees)");
    m_chassisYaw.Append(m_currChassisYaw);
    m_nmTargetAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetAngle(Degrees)");
    m_nmTargetAngle.Append(m_currNmTargetAngle);
    m_nmActualAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualAngle(Degrees)");
    m_nmActualAngle.Append(m_currNmActualAngle);
    m_nmTopTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetTopWheelSpeed(RPM)");
    m_nmTopTarget.Append(m_currNmTopTarget);
    m_nmBottomTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetBottomWheelSpeed(RPM)");
    m_nmBottomTarget.Append(m_currNmBottomTarget);
    m_nmTopActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualTopWheelSpeed(RPM)");
    m_nmTopActual.Append(m_currNmTopActual);
    m_nmBottomActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualBottomWheelSpeed(RPM)");
    m_nmBottomActual.Append(m_currNmBottomActual);
    m_distFromSpeaker = wpi::log::DoubleLogEntry(log, "/NoteManager/DistanceFromSpeaker(Meters)");
    m_distFromSpeaker.Append(m_currDistFromSpeaker);

    m_headingState = wpi::log::StringLogEntry(log, "/Chassis/HeadingState");
    m_headingState.Append(m_currHeadingState);
    m_driveState = wpi::log::StringLogEntry(log, "/Chassis/DriveState");
    m_driveState.Append(m_currDriveState);
    m_noteMgrState = wpi::log::StringLogEntry(log, "/NoteManager/State");
    m_noteMgrState.Append(m_currNoteMgrState);

    m_pose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Chassis/Pose");
    m_pose.Append(m_currPose);

    m_frontLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_frontLeftTarget.Append(m_currFrontLeftTarget);
    m_frontRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontRighttModuleState");
    m_frontRightTarget.Append(m_currFrontRightTarget);
    m_backLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetBackLeftModuleState");
    m_backLeftTarget.Append(m_currBackLeftTarget);
    m_backRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetBackRightModuleState");
    m_backRightTarget.Append(m_currBackRightTarget);
    m_frontLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_frontLeftActual.Append(m_currFrontLeftActual);
    m_frontRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualBackLeftModuleState");
    m_frontRightActual.Append(m_currFrontRightActual);
    m_backLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualBackLeftModuleState");
    m_backLeftActual.Append(m_currBackLeftActual);
    m_backRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualBackRightModuleState");
    m_backRightActual.Append(m_currBackRightActual);

    m_actualSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/ActualSpeed");
    m_actualSpeeds.Append(m_currActualSpeeds);
    m_targetSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/TargetSpeed");
    m_targetSpeeds.Append(m_currActualSpeeds);

    m_frontLeftSwerveModuleslip = wpi::log::BooleanLogEntry(log, "/Chassis/Front Left Sliping");
    m_frontLeftSwerveModuleslip.Append(m_currFrontLeftSwerveModuleslip);
    m_frontRightSwerveModuleslip = wpi::log::BooleanLogEntry(log, "/Chassis/Front Right Sliping");
    m_frontRightSwerveModuleslip.Append(m_currFrontRightSwerveModuleslip);
    m_backLeftSwerveModuleslip = wpi::log::BooleanLogEntry(log, "/Chassis/Back Left Sliping");
    m_backLeftSwerveModuleslip.Append(m_currBackLeftSwerveModuleslip);
    m_backRightSwerveModuleslip = wpi::log::BooleanLogEntry(log, "/Chassis/Back Right Sliping");
    m_backRightSwerveModuleslip.Append(m_currBackRightSwerveModuleslip);
}