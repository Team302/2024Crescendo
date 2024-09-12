
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

#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"

#include "utils/logging/DragonDataLogger.h"
#include "utils/logging/DragonDataLoggerMgr.h"

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(DragonDataLoggerSignals::BoolSignals signalID, bool value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_HAS_VISION:
            if (value != signals->m_currHasVision)
            {
                signals->m_hasVision.Append(value);
                signals->m_currHasVision = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_BACK_SENSOR:
            if (value != signals->m_currBackSensor)
            {
                signals->m_backSensor.Append(value);
                signals->m_currBackSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_FEEDER_SENSOR:
            if (value != signals->m_currFeederSensor)
            {
                signals->m_feederSensor.Append(value);
                signals->m_currFeederSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_FRONT_SENSOR:
            if (value != signals->m_currFrontSensor)
            {
                signals->m_frontSensor.Append(value);
                signals->m_currFrontSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_LAUNCHER_SENSOR:
            if (value != signals->m_currLauncherSensor)
            {
                signals->m_launcherSensor.Append(value);
                signals->m_currLauncherSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_PLACER_IN_SENSOR:
            if (value != signals->m_currPlacerInSensor)
            {
                signals->m_placerInSensor.Append(value);
                signals->m_currPlacerInSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_PLACER_MID_SENSOR:
            if (value != signals->m_currPlacerMidSensor)
            {
                signals->m_placerMidSensor.Append(value);
                signals->m_currPlacerMidSensor = value;
            }
            break;
        case DragonDataLoggerSignals::BoolSignals::NOTE_MANAGER_PLACER_OUT_SENSOR:
            if (value != signals->m_currPlacerOutSensor)
            {
                signals->m_placerOutSensor.Append(value);
                signals->m_currPlacerOutSensor = value;
            }
            break;
        default:
            break;
        }
    }
}

void DragonDataLogger::LogDoubleData(DragonDataLoggerSignals::DoubleSignals signalID, double value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES:
            if (std::abs(value - signals->m_currStoredHeading) > m_doubleTolerance)
            {
                signals->m_storedHeading.Append(value);
                signals->m_currStoredHeading = value;
            }
            break;
        case DragonDataLoggerSignals::DoubleSignals::CHASSIS_YAW_DEGREES:
            if (std::abs(value - signals->m_currChassisYaw) > m_doubleTolerance)
            {
                signals->m_chassisYaw.Append(value);
                signals->m_currChassisYaw = value;
            }
            break;
        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_TARGET_ANGLE_DEGREES:
            if (std::abs(value - signals->m_currNmTargetAngle) > m_doubleTolerance)
            {
                signals->m_nmTargetAngle.Append(value);
                signals->m_currNmTargetAngle = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_ACTUAL_ANGLE_DEGREES:
            if (std::abs(value - signals->m_currNmActualAngle) > m_doubleTolerance)
            {
                signals->m_nmActualAngle.Append(value);
                signals->m_currNmActualAngle = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_TARGET_TOP_WHEEL_SPEED_RPM:
            if (std::abs(value - signals->m_currNmTopTarget) > m_doubleTolerance)
            {
                signals->m_nmTopTarget.Append(value);
                signals->m_currNmTopTarget = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_TARGET_BOTTOM_WHEEL_SPEED_RPM:
            if (std::abs(value - signals->m_currNmBottomTarget) > m_doubleTolerance)
            {
                signals->m_nmBottomTarget.Append(value);
                signals->m_currNmBottomTarget = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_ACTUAL_TOP_WHEEL_SPEED_RPM:
            if (std::abs(value - signals->m_currNmTopActual) > m_doubleTolerance)
            {
                signals->m_nmTopActual.Append(value);
                signals->m_currNmTopActual = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_ACTUAL_BOTTOM_WHEEL_SPEED_RPM:
            if (std::abs(value - signals->m_currNmBottomActual) > m_doubleTolerance)
            {
                signals->m_nmBottomActual.Append(value);
                signals->m_currNmBottomActual = value;
            }
            break;

        case DragonDataLoggerSignals::DoubleSignals::NOTE_MANAGER_DISTANCE_FROM_SPEAKER_METERS:
            if (std::abs(value - signals->m_currDistFromSpeaker) > m_doubleTolerance)
            {
                signals->m_distFromSpeaker.Append(value);
                signals->m_currDistFromSpeaker = value;
            }
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogStringData(DragonDataLoggerSignals::StringSignals signalID, std::string value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::StringSignals::CHASSIS_DRIVE_STATE:
            if (signals->m_currDriveState != value)
            {
                signals->m_driveState.Append(value);
                signals->m_currDriveState = value;
            }
            break;

        case DragonDataLoggerSignals::StringSignals::CHASSIS_HEADING_STATE:
            if (signals->m_currHeadingState != value)
            {
                signals->m_headingState.Append(value);
                signals->m_currHeadingState = value;
            }
            break;

        case DragonDataLoggerSignals::StringSignals::NOTE_MANAGER_STATE:
            if (signals->m_currNoteMgrState != value)
            {
                signals->m_noteMgrState.Append(value);
                signals->m_currNoteMgrState = value;
            }
            break;

        default:
            break;
        }
    }
}
void DragonDataLogger::LogPoseData(DragonDataLoggerSignals::PoseSingals signalID, frc::Pose2d value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_POSE:
            signals->m_pose.Append(value); // always do this as the Pose2d isn't easy to set
            break;

        default:
            break;
        }
    }
}
void DragonDataLogger::LogSwerveModuleStateData(DragonDataLoggerSignals::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_FRONT_STATE:
            if (signals->m_currFrontLeftTarget != value)
            {
                signals->m_frontLeftTarget.Append(value);
                signals->m_currFrontLeftTarget.angle = value.angle;
                signals->m_currFrontLeftTarget.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_LEFT_BACK_STATE:
            if (signals->m_currBackLeftTarget != value)
            {
                signals->m_backLeftTarget.Append(value);
                signals->m_currBackLeftTarget.angle = value.angle;
                signals->m_currBackLeftTarget.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE:
            if (signals->m_currFrontRightTarget != value)
            {
                signals->m_frontRightTarget.Append(value);
                signals->m_currFrontRightTarget.angle = value.angle;
                signals->m_currFrontRightTarget.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::TARGET_RIGHT_BACK_STATE:
            if (signals->m_currBackRightTarget != value)
            {
                signals->m_backRightTarget.Append(value);
                signals->m_currBackRightTarget.angle = value.angle;
                signals->m_currBackRightTarget.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE:
            if (signals->m_currFrontLeftActual != value)
            {
                signals->m_frontLeftActual.Append(value);
                signals->m_currFrontLeftActual.angle = value.angle;
                signals->m_currFrontLeftActual.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE:
            if (signals->m_currBackLeftActual != value)
            {
                signals->m_backLeftActual.Append(value);
                signals->m_currBackLeftActual.angle = value.angle;
                signals->m_currBackLeftActual.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE:
            if (signals->m_currFrontRightActual != value)
            {
                signals->m_frontRightActual.Append(value);
                signals->m_currFrontRightActual.angle = value.angle;
                signals->m_currFrontRightActual.speed = value.speed;
            }
            break;

        case DragonDataLoggerSignals::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE:
            if (signals->m_currBackRightActual != value)
            {
                signals->m_backRightActual.Append(value);
                signals->m_currBackRightActual.angle = value.angle;
                signals->m_currBackRightActual.speed = value.speed;
            }
            break;

        default:
            break;
        }
    }
}

void DragonDataLogger::LogChassisSpeedsData(DragonDataLoggerSignals::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto signals = DragonDataLoggerSignals::GetInstance();
    if (signals != nullptr)
    {
        // TODO:  need to compare/store; need to do element by element
        switch (signalID)
        {
        case DragonDataLoggerSignals::ChassisSpeedSignals::ACTUAL_SPEEDS:
            signals->m_actualSpeeds.Append(value);
            break;

        case DragonDataLoggerSignals::ChassisSpeedSignals::TARGET_SPEEDS:
            signals->m_targetSpeeds.Append(value);
            break;

        default:
            break;
        }
    }
}
