
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

    m_storedHeading = wpi::log::DoubleLogEntry(log, "/Chassis/StoredHeadingDegrees");
    m_storedHeading.Append(m_currStoredHeading);
    m_nmTargetAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetAngleDegrees");
    m_nmTargetAngle.Append(m_currNmTargetAngle);
    m_nmActualAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualAngleDegrees");
    m_nmActualAngle.Append(m_currNmActualAngle);
    m_nmTopTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetTopWheelSpeedRevPerSec");
    m_nmTopTarget.Append(m_currNmTopTarget);
    m_nmBottomTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetBottomWheelSpeedRevPerSec");
    m_nmBottomTarget.Append(m_currNmBottomTarget);
    m_nmTopActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualTopWheelSpeedRevPerSec");
    m_nmTopActual.Append(m_currNmTopActual);
    m_nmBottomActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualBottomWheelSpeedRevPerSec");
    m_nmBottomActual.Append(m_currNmBottomActual);
    m_distFromSpeaker = wpi::log::DoubleLogEntry(log, "/NoteManager/DistanceFromSpeakerMeters");
    m_distFromSpeaker.Append(m_currDistFromSpeaker);

    m_hasVision = wpi::log::BooleanLogEntry(log, "/NoteManager/HasVision");
    m_hasVision.Append(m_currHasVision);

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
    m_frontRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_frontRightTarget.Append(m_currFrontRightTarget);
    m_backLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_backLeftTarget.Append(m_currBackLeftTarget);
    m_backRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_backRightTarget.Append(m_currBackRightTarget);
    m_frontLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_frontLeftActual.Append(m_currFrontLeftActual);
    m_frontRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_frontRightActual.Append(m_currFrontRightActual);
    m_backLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_backLeftActual.Append(m_currBackLeftActual);
    m_backRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_backRightActual.Append(m_currBackRightActual);

    m_actualSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/ActualSpeed");
    m_actualSpeeds.Append(m_currActualSpeeds);
    m_targetSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/TargetSpeed");
    m_targetSpeeds.Append(m_currActualSpeeds);
}