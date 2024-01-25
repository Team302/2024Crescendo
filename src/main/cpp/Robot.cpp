// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotController.h>
#include <Robot.h>

#include <string>

#include <cameraserver/CameraServer.h>

#include <auton/AutonPreviewer.h>
#include <auton/CyclePrimitives.h>
#include <chassis/holonomic/HolonomicDrive.h>
#include "chassis/swerve/SwerveChassis.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include <driveteamfeedback/DriverFeedback.h>
#include <PeriodicLooper.h>
#include <Robot.h>
#include <robotstate/RobotState.h>
#include "teleopcontrol/TeleopControl.h"
#include <utils/DragonField.h>
#include <utils/FMSData.h>
#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>
#include <utils/WaypointXmlParser.h>
#include <utils/BuildDetailsReader.h>
#include <RobotDefinitions.h>

#include <AdjustableItemMgr.h>
/// DEBUGGING

#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include "configs/RobotElementNames.h"

/* How to check robot variant
#if ROBOT_VARIANT == 2024
#warning COMP BOT
#else
#warning UNKNOWN
#endif
*/

using namespace std;

void Robot::RobotInit()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("arrived"));

    m_controller = nullptr;

    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    // Build the robot
    RobotConfigMgr::GetInstance()->InitRobot((RobotConfigMgr::RobotIdentifier)teamNumber);
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();

    auto waypointParser = WaypointXmlParser::GetInstance();
    waypointParser->ParseWaypoints();

    // Get AdjustableItemMgr instance
    m_tuner = nullptr;
    // m_tuner = AdjustableItemMgr::GetInstance();

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();

    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    m_holonomic = nullptr;
    if (m_chassis != nullptr)
    {
        m_holonomic = new HolonomicDrive();
    }

    m_cyclePrims = new CyclePrimitives();
    m_previewer = new AutonPreviewer(m_cyclePrims); // TODO:: Move to DriveTeamFeedback
    m_field = DragonField::GetInstance();           // TODO: move to drive team feedback

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("end"));
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    LoggableItemMgr::GetInstance()->LogData();
    Logger::GetLogger()->PeriodicLog();

    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

    // ToDo:: Move to DriveTeamFeedback
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }
    if (m_field != nullptr && m_chassis != nullptr)
    {
        m_field->UpdateRobotPosition(m_chassis->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
    }

    // m_tuner->ListenForUpdates();

    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("arrived"));

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("end"));
}

void Robot::AutonomousPeriodic()
{
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

void Robot::TeleopInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("arrived"));

    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("after teleopcontrol"), string("arrived"));

    if (m_chassis != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("have a swerve chassis"), string("arrived"));
    }
    if (m_controller != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("have a controller"), string("arrived"));
    }
    if (m_chassis != nullptr && m_controller != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("chassis and controller"), string("arrived"));
        if (m_holonomic != nullptr)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("holonomic "), string("arrived"));
            m_holonomic->Init();
        }

        // Create chassismovement to flush out any drive options from auton
        ChassisMovement resetMoveInfo;
        resetMoveInfo.driveOption = ChassisOptionEnums::DriveStateType::FIELD_DRIVE;
        resetMoveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("reset chassis movement"), string("arrived"));

        m_chassis->Drive();
        dynamic_cast<VisionDrive *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::VISION_DRIVE))->setInAutonMode(false);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("drive"), string("arrived"));
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    // now in teleop, clear field of trajectories
    if (m_field != nullptr)
    {
        m_field->ResetField(); // ToDo:  Move to DriveTeamFeedback
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("end"));
}

void Robot::TeleopPeriodic()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("arrived"));
    if (m_chassis != nullptr && m_controller != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicRun"), string("arrived"));
        if (m_holonomic != nullptr)
        {
            m_holonomic->Run();
        }
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("end"));
}

void Robot::DisabledInit()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("arrived"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("end"));
}

void Robot::DisabledPeriodic()
{
}

void Robot::TestInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TestInit"), string("arrived"));
}

void Robot::TestPeriodic()
{
}

void Robot::SimulationInit()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::SimulationPeriodic()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
