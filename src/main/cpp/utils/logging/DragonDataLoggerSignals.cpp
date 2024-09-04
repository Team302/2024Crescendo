
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
    m_nmTargetAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetAngleDegrees");
    m_nmActualAngle = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualAngleDegrees");
    m_nmTopTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetTopWheelSpeedRevPerSec");
    m_nbBottomTarget = wpi::log::DoubleLogEntry(log, "/NoteManager/TargetBottomWheelSpeedRevPerSec");
    m_nmTopActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualTopWheelSpeedRevPerSec");
    m_nmBottomActual = wpi::log::DoubleLogEntry(log, "/NoteManager/ActualBottomWheelSpeedRevPerSec");
    m_distFromSpeaker = wpi::log::DoubleLogEntry(log, "/NoteManager/DistanceFromSpeakerMeters");

    m_hasVision = wpi::log::BooleanLogEntry(log, "/NoteManager/HasVision");

    m_headingState = wpi::log::StringLogEntry(log, "/Chassis/HeadingState");
    m_driveState = wpi::log::StringLogEntry(log, "/Chassis/DriveState");
    m_noteMgrState = wpi::log::StringLogEntry(log, "/NoteManager/State");

    m_pose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Chassis/Pose");
    m_frontLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_frontRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_backLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_backRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/TargetFrontLeftModuleState");
    m_frontLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_frontRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_backLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");
    m_backRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/ActualFrontLeftModuleState");

    m_actualSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/ActualSpeed");
    m_targetSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/TargetSpeed");
}