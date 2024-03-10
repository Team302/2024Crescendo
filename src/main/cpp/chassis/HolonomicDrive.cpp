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
#include <algorithm>
#include <string>

// FRC includes
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "frc/kinematics/ChassisSpeeds.h"

// Team 302 Includes
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/HolonomicDrive.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVision.h"
#include "utils/logging/Logger.h"
#include "chassis/driveStates/DriveToNote.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"

using std::string;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   m_swerve(ChassisConfigMgr::GetInstance()->GetCurrentConfig() != nullptr ? ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis() : nullptr),
                                   m_previousDriveState(ChassisOptionEnums::DriveStateType::FIELD_DRIVE),
                                   m_checkTippingLatch(false)
{
    Init();
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    InitChassisMovement();
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{

    auto controller = TeleopControl::GetInstance();
    if (controller != nullptr && m_swerve != nullptr)
    {
        auto forward = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

        InitSpeeds(forward, strafe, rotate);

        // teleop buttons to check for mode changes
        auto isResetPoseSelected = controller->IsButtonPressed(TeleopControlFunctions::RESET_POSITION);
        auto isAlignGamePieceSelected = controller->IsButtonPressed(TeleopControlFunctions::INTAKE);
        auto isRobotOriented = controller->IsButtonPressed(TeleopControlFunctions::ROBOT_ORIENTED_DRIVE);
        auto isAlignWithSpeakerSelected = controller->IsButtonPressed(TeleopControlFunctions::AUTO_SPEAKER);
        auto isAlignWithStageSelected = controller->IsButtonPressed(TeleopControlFunctions::AUTO_STAGE);
        auto isAlignWithAmpSelected = controller->IsButtonPressed(TeleopControlFunctions::AUTO_AMP);
        auto isHoldPositionSelected = controller->IsButtonPressed(TeleopControlFunctions::HOLD_POSITION);
        auto isFaceForward = controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_FORWARD);
        auto isFaceBackward = controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_BACKWARD);
        auto isSlowMode = controller->IsButtonPressed(TeleopControlFunctions::SLOW_MODE);
        auto checkTipping = controller->IsButtonPressed(TeleopControlFunctions::TIPCORRECTION_TOGGLE);

        // Switch Heading Option and Drive Mode
        if (isAlignGamePieceSelected)
        {
            StateMgr *noteStateManager = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::NOTE_MANAGER);
            auto noteMgr = noteStateManager != nullptr ? dynamic_cast<noteManagerGen *>(noteStateManager) : nullptr;
            auto vision = DragonVision::GetDragonVision();
            if (vision != nullptr)
            {
                if (!noteMgr->getfrontIntakeSensor()->Get() && !noteMgr->getbackIntakeSensor()->Get() && vision->GetVisionData(DragonVision::VISION_ELEMENT::NOTE).has_value())
                    AlignGamePiece();
                else
                    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
            }
        }
        else if (isAlignWithAmpSelected)
        {
            AlignToAmp();
        }
        else if (isAlignWithStageSelected || m_stageDrive)
        {
            AlignToStage();
            m_previousDriveState = m_moveInfo.driveOption;
            m_stageDrive = true;
            m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::STAGE_DRIVE;
        }
        else if (isAlignWithSpeakerSelected)
        {
            AlignToSpeaker();
        }
        else
        {
            // Switch Heading Options
            if (isResetPoseSelected)
            {
                ResetPose();
            }
            else if (isFaceForward)
            {
                TurnForward();
            }
            else if (isFaceBackward)
            {
                TurnBackward();
            }
            // Switch Drive Modes
            if (isHoldPositionSelected)
            {
                HoldPosition();
            }
            else if (isRobotOriented || m_robotOrientedDrive)
            {
                CheckRobotOriented(isRobotOriented);
                if (m_robotOrientedDrive)
                {
                    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;
                }
                else
                {
                    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
                }
            }
            else
            {
                if ((m_moveInfo.driveOption != ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER))
                {
                    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
                    if ((abs(forward) < 0.05 && abs(strafe) < 0.05 && abs(rotate) < 0.05) && (m_moveInfo.headingOption != ChassisOptionEnums::HeadingOption::FACE_SPEAKER))
                    {
                        m_previousDriveState = m_moveInfo.driveOption;
                        m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::STOP_DRIVE;
                    }
                }
            }
        }

        if (isSlowMode)
        {
            SlowMode();
        }

        if (abs(rotate) > 0.05)
        {
            m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
            m_stageDrive = false;
        }

        CheckTipping(checkTipping);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Heading Option", m_moveInfo.headingOption);
        m_swerve->Drive(m_moveInfo);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("HolonomicDrive"), string("Run"), string("nullptr"));
    }
}

void HolonomicDrive::InitChassisMovement()
{
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    m_moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    m_moveInfo.pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    m_moveInfo.centerOfRotationOffset = frc::Translation2d();
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    m_moveInfo.noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    m_moveInfo.yawAngle = units::angle::degree_t(0.0);
    m_moveInfo.checkTipping = false;
    m_moveInfo.tippingTolerance = units::angle::degree_t(5.0);
    m_moveInfo.tippingCorrection = -0.1;
}

void HolonomicDrive::InitSpeeds(double forwardScale,
                                double strafeScale,
                                double rotateScale)
{
    auto maxSpeed = m_swerve->GetMaxSpeed();
    auto maxAngSpeed = m_swerve->GetMaxAngularSpeed();
    // auto scale = (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue) ? 1.0 : -1.0;
    m_moveInfo.chassisSpeeds.vx = forwardScale * maxSpeed * -1.0;
    m_moveInfo.chassisSpeeds.vy = strafeScale * maxSpeed * -1.0;
    m_moveInfo.chassisSpeeds.omega = rotateScale * maxAngSpeed;

    if ((abs(forwardScale) > 0.0) || (abs(strafeScale) > 0.0) || (abs(rotateScale) > 0.0))
    {
        m_moveInfo.pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    }
}

void HolonomicDrive::ResetPose()
{
    m_swerve->ResetYaw();
}
void HolonomicDrive::AlignGamePiece()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
}
void HolonomicDrive::AlignToStage()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_STAGE;
}
void HolonomicDrive::AlignToSpeaker()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_SPEAKER;
}
void HolonomicDrive::AlignToAmp()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
    m_moveInfo.yawAngle = units::angle::degree_t(-90.0);
}
void HolonomicDrive::HoldPosition()
{
    m_previousDriveState = m_moveInfo.driveOption;
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::HOLD_DRIVE;
}
void HolonomicDrive::DriveToGamePiece()
{
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE;
}
void HolonomicDrive::TurnForward()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
    {
        m_moveInfo.yawAngle = units::angle::degree_t(0.0);
    }
    else
    {
        m_moveInfo.yawAngle = units::angle::degree_t(180.0);
    }
}
void HolonomicDrive::TurnBackward()
{
    m_moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;

    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
    {
        m_moveInfo.yawAngle = units::angle::degree_t(180.0);
    }
    else
    {
        m_moveInfo.yawAngle = units::angle::degree_t(0.0);
    }
}
void HolonomicDrive::SlowMode()
{
    m_moveInfo.chassisSpeeds.vx *= m_slowModeMultiplier;
    m_moveInfo.chassisSpeeds.vy *= m_slowModeMultiplier;
    m_moveInfo.chassisSpeeds.omega *= m_slowModeMultiplier;
}

void HolonomicDrive::CheckTipping(bool isSelected)
{
    if (isSelected)
    {
        if (m_checkTippingLatch == false)
        {
            m_CheckTipping = !m_CheckTipping;
            m_checkTippingLatch = true;
        }
    }
    else
    {
        m_checkTippingLatch = false;
    }
    m_moveInfo.checkTipping = m_CheckTipping;
}

void HolonomicDrive::CheckRobotOriented(bool isSelected)
{
    if (isSelected)
    {
        if (!m_robotOrientedLatch)
        {
            m_robotOrientedDrive = !m_robotOrientedDrive;
            m_robotOrientedLatch = true;
        }
    }
    else
    {
        m_robotOrientedLatch = false;
    }
}

void HolonomicDrive::Exit()
{
}

/// @brief indicates that we are not at our target
/// @return bool
bool HolonomicDrive::AtTarget()
{
    return false;
}
