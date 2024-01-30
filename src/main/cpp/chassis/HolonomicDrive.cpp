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
#include "chassis/driveStates/VisionDrive.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/FMSData.h"
#include "utils/logging/Logger.h"

using std::string;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   m_swerve(ChassisConfigMgr::GetInstance()->GetCurrentConfig() != nullptr ? ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis() : nullptr),
                                   m_previousDriveState(ChassisOptionEnums::DriveStateType::FIELD_DRIVE),
                                   m_checkTippingLatch(false)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::HolonomicDrive"), string("arrived"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::HolonomicDrive"), string("end"));
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Init"), string("arrived"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Init"), string("end"));
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Run"), string("arrived"));
    auto controller = TeleopControl::GetInstance();
    if (controller != nullptr && m_swerve != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("controller "), controller != nullptr ? string("not nullptr ") : string("nullptr"));

        auto forward = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("forward"), forward);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("strafe"), strafe);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("rotate"), rotate);
        ChassisMovement moveInfo = InitChassisMovement(forward, strafe, rotate);

        // teleop buttons to check for mode changes
        auto isResetPoseSelected = controller->IsButtonPressed(TeleopControlFunctions::RESET_POSITION);
        auto isAlignGamePieceSelected = controller->IsButtonPressed(TeleopControlFunctions::ALIGN_FLOOR_GAME_PIECE);
        auto isHoldPositionSelected = controller->IsButtonPressed(TeleopControlFunctions::HOLD_POSITION);
        auto isFaceForward = controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_FORWARD);
        auto isFaceBackward = controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_BACKWARD);
        auto isSlowMode = controller->IsButtonPressed(TeleopControlFunctions::SLOW_MODE);
        auto checkTipping = controller->IsButtonPressed(TeleopControlFunctions::TIPCORRECTION_TOGGLE);

        // Switch Heading Option and Drive Mode
        if (isAlignGamePieceSelected)
        {
            AlignGamePiece(moveInfo);
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
                TurnForward(moveInfo);
            }
            else if (isFaceBackward)
            {
                TurnBackward(moveInfo);
            }
            // Switch Drive Modes
            if (isHoldPositionSelected)
            {
                HoldPosition(moveInfo);
            }
            else
            {
                if ((abs(forward) > 0.05 || abs(strafe) > 0.05 || abs(rotate) > 0.05) && !m_inVisionDrive)
                {
                    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
                    m_previousDriveState = moveInfo.driveOption;
                }
            }
        }

        if (isSlowMode)
        {
            SlowMode(moveInfo);
        }

        CheckTipping(checkTipping, moveInfo);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("vx"), moveInfo.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("vy"), moveInfo.chassisSpeeds.vy.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("omega"), moveInfo.chassisSpeeds.omega.to<double>());

        m_swerve->Drive(moveInfo);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Run"), string("end"));
}

ChassisMovement HolonomicDrive::InitChassisMovement(double forwardScale,
                                                    double strafeScale,
                                                    double rotateScale)
{
    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

    auto maxSpeed = m_swerve->GetMaxSpeed();
    auto maxAngSpeed = m_swerve->GetMaxAngularSpeed();
    moveInfo.chassisSpeeds.vx = forwardScale * maxSpeed;
    moveInfo.chassisSpeeds.vy = strafeScale * maxSpeed;
    moveInfo.chassisSpeeds.omega = rotateScale * maxAngSpeed;

    return moveInfo;
}

void HolonomicDrive::ResetPose()
{
    m_swerve->ResetYaw();
}
void HolonomicDrive::AlignGamePiece(ChassisMovement &moveInfo)
{
    m_inVisionDrive = true;
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::VISION_DRIVE;
}
void HolonomicDrive::HoldPosition(ChassisMovement &moveInfo)
{
    NonVisionDrive(moveInfo);
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::HOLD_DRIVE;
    m_previousDriveState = moveInfo.driveOption;
}
void HolonomicDrive::TurnForward(ChassisMovement &moveInfo)
{
    NonVisionDrive(moveInfo);
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
    {
        moveInfo.yawAngle = units::angle::degree_t(0.0);
    }
    else
    {
        moveInfo.yawAngle = units::angle::degree_t(180.0);
    }
}
void HolonomicDrive::TurnBackward(ChassisMovement &moveInfo)
{
    NonVisionDrive(moveInfo);
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;

    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
    {
        moveInfo.yawAngle = units::angle::degree_t(180.0);
    }
    else
    {
        moveInfo.yawAngle = units::angle::degree_t(0.0);
    }
}
void HolonomicDrive::SlowMode(ChassisMovement &moveInfo)
{
    NonVisionDrive(moveInfo);
    moveInfo.chassisSpeeds.vx *= m_slowModeMultiplier;
    moveInfo.chassisSpeeds.vy *= m_slowModeMultiplier;
    moveInfo.chassisSpeeds.omega *= m_slowModeMultiplier;
}

void HolonomicDrive::NonVisionDrive(ChassisMovement &moveInfo)
{
    // no longer in vision drive, set boolean and reset offsets in VisionDrive
    m_inVisionDrive = false;
    auto visionDrive = dynamic_cast<VisionDrive *>(m_swerve->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::VISION_DRIVE));
    visionDrive->ResetVisionDrive();
}

void HolonomicDrive::CheckTipping(bool isSelected, ChassisMovement &moveInfo)
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
    moveInfo.checkTipping = m_CheckTipping;
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
