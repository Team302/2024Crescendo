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
#include <memory>
#include <string>

// FRC includes
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include <frc/kinematics/ChassisSpeeds.h>

// Team 302 Includes
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include <chassis/holonomic/HolonomicDrive.h>
#include <chassis/IChassis.h>
#include <gamepad/IDragonGamepad.h>
#include "teleopcontrol/TeleopControl.h"
#include <teleopcontrol/TeleopControlFunctions.h>
#include "State.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "utils/logging/Logger.h"
#include <chassis/swerve/driveStates/DragonTrajectoryGenerator.h>
#include <utils/DragonField.h>
#include <DragonVision/DragonVision.h>
#include <chassis/swerve/driveStates/VisionDrive.h>
#include <robotstate/RobotState.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   IRobotStateChangeSubscriber(),
                                   m_chassis(nullptr),
                                   m_swerve(nullptr),
                                   m_previousDriveState(ChassisOptionEnums::DriveStateType::FIELD_DRIVE),
                                   m_desiredGamePiece(RobotStateChanges::GamePiece::None)
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_swerve = config != nullptr ? config->GetSwerveChassis() : nullptr;
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
}

void HolonomicDrive::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::DesiredGamePiece)
    {
        m_desiredGamePiece = static_cast<RobotStateChanges::GamePiece>(value);
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Initialized?"), "true");
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{
    auto controller = TeleopControl::GetInstance();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("controller "), controller != nullptr ? string("not nullptr ") : string("nullptr"));
    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("DriveOptionBEGINNING"), moveInfo.driveOption);
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("chassis "), m_chassis != nullptr ? string("not nullptr ") : string("nullptr"));
    if (controller != nullptr && m_chassis != nullptr)
    {
        moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

        if (controller->IsButtonPressed(TeleopControlFunctions::RESET_POSITION) && !m_hasResetPosition)
        {
            if (m_swerve != nullptr)
            {
                m_swerve->ResetYaw();
            }

            m_hasResetPosition = true;
        }
        else
        {
            m_hasResetPosition = false;
        }

        m_findingFloorGamePiece = false;

        bool alignFloorPiece = controller->IsButtonPressed(TeleopControlFunctions::ALIGN_FLOOR_GAME_PIECE);
        bool alignAprilTag = controller->IsButtonPressed(TeleopControlFunctions::ALIGN_APRIL_TAG);
        bool alignSubstation = controller->IsButtonPressed(TeleopControlFunctions::ALIGN_SUBSTATION_GAME_PIECE);

        if (alignFloorPiece || alignSubstation || alignAprilTag)
        {
            m_inVisionDrive = true;

            if (alignFloorPiece || alignSubstation)
            {
                // set pipeline to discover retroreflective
                if (m_desiredGamePiece == RobotStateChanges::GamePiece::Cube)
                    // visionapi - revisit this with me dragonvision
                    DragonVision::GetDragonVision()->SetPipeline(DragonCamera::PIPELINE::MACHINE_LEARNING, DragonVision::CAMERA_POSITION::BACK_INTAKE);
                moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
                if (alignFloorPiece)
                    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::VISION_DRIVE;

                m_findingFloorGamePiece = true;
            }

            if (controller->IsButtonPressed(TeleopControlFunctions::ALIGN_APRIL_TAG))
            {
                // set pipeline to discover april tags

                DragonVision::GetDragonVision()->SetPipeline(DragonCamera::PIPELINE::APRIL_TAG, DragonVision::CAMERA_POSITION::FRONT);
                DragonVision::GetDragonVision()->SetPipeline(DragonCamera::PIPELINE::APRIL_TAG, DragonVision::CAMERA_POSITION::BACK);
                moveInfo.headingOption = ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG;
                moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
            }
        }
        else
        {
            // no longer in vision drive, set boolean and reset offsets in VisionDrive
            m_inVisionDrive = false;
            auto visionDrive = dynamic_cast<VisionDrive *>(m_swerve->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::VISION_DRIVE));

            visionDrive->ResetVisionDrive();
        }

        // update leds based on finding cube with vision
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::FindingCube, m_findingFloorGamePiece ? 1 : 0);

        if (controller->IsButtonPressed(TeleopControlFunctions::HOLD_POSITION))
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::HOLD_DRIVE;
            m_previousDriveState = moveInfo.driveOption;
        }
        if (controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_FORWARD))
        {
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
        if (controller->IsButtonPressed(TeleopControlFunctions::AUTO_TURN_BACKWARD))
        {
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

        auto maxSpeed = m_chassis->GetMaxSpeed();
        auto maxAngSpeed = m_chassis->GetMaxAngularSpeed();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("maxspeed "), maxSpeed.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("maxangspeed "), maxAngSpeed.to<double>());

        auto forward = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

        if (controller->IsButtonPressed(TeleopControlFunctions::SLOW_MODE))
        {
            forward *= m_slowModeMultiplier;
            strafe *= m_slowModeMultiplier;
            rotate *= m_slowModeMultiplier;
        }

        if ((abs(forward) > 0.05 || abs(strafe) > 0.05 || abs(rotate) > 0.05) && !m_inVisionDrive)
        {
            moveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
            m_previousDriveState = moveInfo.driveOption;
        }

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("axis read"));

        if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed &&
            moveInfo.driveOption == ChassisOptionEnums::DriveStateType::FIELD_DRIVE)
        {
            forward *= -1.0;
            strafe *= -1.0;
        }

        moveInfo.chassisSpeeds.vx = forward * maxSpeed;
        moveInfo.chassisSpeeds.vy = strafe * maxSpeed;
        moveInfo.chassisSpeeds.omega = rotate * maxAngSpeed;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vx"), moveInfo.chassisSpeeds.vx.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Vy"), moveInfo.chassisSpeeds.vy.to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("DriveOptionEND"), moveInfo.driveOption);

        if (controller->IsButtonPressed(TeleopControlFunctions::TIPCORRECTION_TOGGLE))
        {
            if (m_latch == false)
            {
                m_CheckTipping = !m_CheckTipping;
                m_latch = true;
            }
        }
        else
        {
            m_latch = false;
        }

        moveInfo.checkTipping = m_CheckTipping;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("check tipping"), moveInfo.checkTipping);
        /// debugging
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ANickDebugging"), string("DriveState"), moveInfo.driveOption);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ANickDebugging"), string("HeadingOption"), moveInfo.headingOption);

        m_chassis->Drive(moveInfo);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));
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

bool HolonomicDrive::IsAutoAligning()
{
    bool isAutoAligning = false;

    auto controller = TeleopControl::GetInstance();

    // Check if we are trying to align to any of the grids
    isAutoAligning = controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_LEFT_GRID) ||
                     controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_CENTER_GRID) ||
                     controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_RIGHT_GRID);

    return isAutoAligning;
}

std::pair<ChassisOptionEnums::RELATIVE_POSITION, ChassisOptionEnums::RELATIVE_POSITION> HolonomicDrive::GetAutoAlignDestination()
{
    // create default destination
    std::pair<ChassisOptionEnums::RELATIVE_POSITION, ChassisOptionEnums::RELATIVE_POSITION> destination = {ChassisOptionEnums::RELATIVE_POSITION::CENTER,
                                                                                                           ChassisOptionEnums::RELATIVE_POSITION::CENTER};

    auto controller = TeleopControl::GetInstance();

    // check for desired grid first
    if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_LEFT_GRID))
    {
        destination.first = ChassisOptionEnums::RELATIVE_POSITION::LEFT;
    }
    else if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_CENTER_GRID))
    {
        destination.first = ChassisOptionEnums::RELATIVE_POSITION::CENTER;
    }
    else if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_RIGHT_GRID))
    {
        destination.first = ChassisOptionEnums::RELATIVE_POSITION::RIGHT;
    }

    // next, check for desired column/node
    if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_LEFT_NODE))
    {
        destination.second = ChassisOptionEnums::RELATIVE_POSITION::LEFT;
    }
    else if (controller->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_RIGHT_NODE))
    {
        destination.second = ChassisOptionEnums::RELATIVE_POSITION::RIGHT;
    }
    else
    {
        destination.second = ChassisOptionEnums::RELATIVE_POSITION::CENTER;
    }

    // finally, return desired destination
    return destination;
}