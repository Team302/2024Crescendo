// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotController.h>
#include <Robot.h>

#include <string>

#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/HolonomicDrive.h"
#include "chassis/SwerveChassis.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "driveteamfeedback/DriverFeedback.h"
#include "mechanisms/noteManager/generated/noteManagerGen.h"
#include "mechanisms/ClimberManager/generated/ClimberManagerGen.h"
#include "PeriodicLooper.h"
#include "Robot.h"
#include "robotstate/RobotState.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "utils/logging/DragonDataLoggerMgr.h"
#include "utils/logging/LoggableItemMgr.h"
#include "utils/logging/Logger.h"
#include "utils/logging/LoggerData.h"
#include "utils/logging/LoggerEnums.h"
#include "DragonVision/DragonVision.h"
#include "utils/logging/DataTrace.h"

using std::string;

void Robot::RobotInit()
{
    isFMSAttached = frc::DriverStation::IsFMSAttached();

    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("arrived"));
        InitializeDataTracing();
    }

    m_controller = nullptr;

    InitializeRobot();
    InitializeAutonOptions();
    InitializeDriveteamFeedback();

    m_datalogger = DragonDataLoggerMgr::GetInstance();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("RobotInit"), string("end"));
    }
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
    isFMSAttached = isFMSAttached ? true : frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        LoggableItemMgr::GetInstance()->LogData();
        Logger::GetLogger()->PeriodicLog();
    }

    if (m_datalogger != nullptr && !frc::DriverStation::IsDisabled())
    {
        m_datalogger->PeriodicDataLog();
    }

    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

    UpdateDriveTeamFeedback();
    LogDiagnosticData();
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
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("arrived"));
    }

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousInit"), string("end"));
    }
}

void Robot::AutonomousPeriodic()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousPeriodic"), string("arrived"));
    }

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("AutonomousPeriodic"), string("end"));
    }
}

void Robot::TeleopInit()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("arrived"));
    }

    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Init();
    }

    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();

    if (config != nullptr)
    {
        auto stateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER);
        auto noteMgr = stateMgr != nullptr ? dynamic_cast<noteManagerGen *>(stateMgr) : nullptr;

        if (noteMgr != nullptr)
        {
            bool allSensorsOff = ((!noteMgr->getfeederSensor()->Get()) &&
                                  (!noteMgr->getlauncherSensor()->Get()) &&
                                  (!noteMgr->getplacerInSensor()->Get()) &&
                                  (!noteMgr->getplacerMidSensor()->Get()) &&
                                  (!noteMgr->getplacerOutSensor()->Get()) &&
                                  (!noteMgr->getbackIntakeSensor()->Get()) &&
                                  (!noteMgr->getfrontIntakeSensor()->Get()));
            if (stateMgr != nullptr && allSensorsOff)
            {
                stateMgr->SetCurrentState(noteManagerGen::STATE_NAMES::STATE_READY, true);
            }
        }
        auto climberMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER);
        if (climberMgr != nullptr)
        {
            climberMgr->SetCurrentState(ClimberManagerGen::STATE_NAMES::STATE_HOLD, true);
        }
    }

    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopInit"), string("end"));
    }
}

void Robot::TeleopPeriodic()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("arrived"));
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Run();
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();

    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TeleopPeriodic"), string("end"));
    }
}

void Robot::DisabledInit()
{
    if (!isFMSAttached)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("arrived"));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("DisabledInit"), string("end"));
    }
}

void Robot::DisabledPeriodic()
{
    // TODO Make method in DragonVision for next year
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();

        auto visionPosition = vision->GetRobotPosition();
        auto hasVisionPose = visionPosition.has_value();
        if (hasVisionPose)
        {
            auto initialRot = visionPosition.value().estimatedPose.ToPose2d().Rotation().Degrees();

            // use the path angle as an initial guess for the MegaTag2 calc; chassis is most-likely 0.0 right now which may cause issues based on color
            auto megaTag2Position = vision->GetRobotPositionMegaTag2(initialRot, // chassis->GetYaw(), // mtAngle.Degrees(),
                                                                     units::angular_velocity::degrees_per_second_t(0.0),
                                                                     units::angle::degree_t(0.0),
                                                                     units::angular_velocity::degrees_per_second_t(0.0),
                                                                     units::angle::degree_t(0.0),
                                                                     units::angular_velocity::degrees_per_second_t(0.0));

            if (megaTag2Position.has_value())
            {
                chassis->SetYaw(initialRot);
                chassis->SetStoredHeading(initialRot);
                chassis->ResetPose(megaTag2Position.value().estimatedPose.ToPose2d());
            }
            else if (hasVisionPose)
            {
                chassis->SetYaw(initialRot);
                chassis->SetStoredHeading(initialRot);
                chassis->ResetPose(visionPosition.value().estimatedPose.ToPose2d());
            }
        }
    }
}
void Robot::TestInit()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("TestInit"), string("arrived"));
}

void Robot::TestPeriodic()
{
}

void Robot::LogDiagnosticData()
{
    const unsigned int loggingEveyNloops = 20;
    static unsigned int loopCounter = 0;

    unsigned int step = loopCounter % loggingEveyNloops;

    if (step == 0)
        LogSensorData();
    else if (step == 1)
        LogMotorData();
    else if (step == 3)
        LogCameraData();
    else if (step == 4)
    {
        m_chassis->LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES::LEFT_BACK);
        m_chassis->LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES::RIGHT_BACK);
        m_chassis->LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES::LEFT_FRONT);
        m_chassis->LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES::RIGHT_FRONT);
    }
    loopCounter++;
}

void Robot::LogSensorData()
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();

    if (config != nullptr)
    {
        auto stateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER);
        auto noteMgr = stateMgr != nullptr ? dynamic_cast<noteManagerGen *>(stateMgr) : nullptr;
        if (noteMgr != nullptr)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ("SensorsIntake"), string("Front Intake"), noteMgr->getfrontIntakeSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsIntake"), string("Back Intake"), noteMgr->getbackIntakeSensor()->Get());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsLauncher"), string("Feeder"), noteMgr->getfeederSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsLauncher"), string("Launcher"), noteMgr->getlauncherSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsLauncher"), string("LauncherAngleHomeSwitch"), noteMgr->getlauncherAngle()->IsReverseLimitSwitchClosed());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsPlacer"), string("PlacerIn"), noteMgr->getplacerInSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsPlacer"), string("PlacerMid"), noteMgr->getplacerMidSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsPlacer"), string("PlacerOut"), noteMgr->getplacerOutSensor()->Get());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("SensorsPlacer"), string("PlacerHomeSwitch"), noteMgr->getPlacer()->IsReverseLimitSwitchClosed());
        }
    }
}

void Robot::LogMotorData()
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();

    if (config != nullptr)
    {
        auto stateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER);
        auto noteMgr = stateMgr != nullptr ? dynamic_cast<noteManagerGen *>(stateMgr) : nullptr;
        if (noteMgr != nullptr)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ("MotorDiagnosticsIntake"), string("Front Intake"), noteMgr->getfrontIntake()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsIntake"), string("Back Intake"), noteMgr->getbackIntake()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsIntake"), string("Transfer"), noteMgr->getTransfer()->GetCounts());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsPlacer"), string("Elevator"), noteMgr->getElevator()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsPlacer"), string("Placer"), noteMgr->getPlacer()->GetCounts());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsLauncher"), string("Feeder"), noteMgr->getFeeder()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsLauncher"), string("Launcher angle"), units::angle::degree_t(noteMgr->getlauncherAngleEncoder()->GetAbsolutePosition()).value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsLauncher"), string("Launcher top"), noteMgr->getlauncherTop()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsLauncher"), string("Launcher bottom"), noteMgr->getlauncherBottom()->GetCounts());
        }

        stateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER);
        auto climberMgr = stateMgr != nullptr ? dynamic_cast<ClimberManagerGen *>(stateMgr) : nullptr;
        if (climberMgr != nullptr)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsClimber"), string("Left climber"), climberMgr->getleftClimber()->GetCounts());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MotorDiagnosticsClimber"), string("Right climber"), climberMgr->getrightClimber()->GetCounts());
        }
    }
}

void Robot::LogCameraData()
{
    // TODO: implement encoder logging for chassis
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("LimelightDiagnostics"), string("LAUNCHER Connected"), DragonVision::GetDragonVision()->HealthCheck(RobotElementNames::CAMERA_USAGE::LAUNCHE));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("LimelightDiagnostics"), string("PLACER INTAKE Connected"), DragonVision::GetDragonVision()->HealthCheck(RobotElementNames::CAMERA_USAGE::PINTAKE));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("LimelightDiagnostics"), string("LAUNCHER INTAKE Connected"), DragonVision::GetDragonVision()->HealthCheck(RobotElementNames::CAMERA_USAGE::LINTAKE));
}

void Robot::SimulationInit()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::SimulationPeriodic()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::InitializeRobot()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    RobotConfigMgr::GetInstance()->InitRobot((RobotConfigMgr::RobotIdentifier)teamNumber);
    ChassisConfigMgr::GetInstance()->InitChassis(static_cast<RobotConfigMgr::RobotIdentifier>(teamNumber));
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
    m_holonomic = nullptr;
    if (m_chassis != nullptr)
    {
        m_holonomic = new HolonomicDrive();
    }

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();
}

void Robot::InitializeAutonOptions()
{
    m_cyclePrims = new CyclePrimitives(); // intialize auton selections
    m_previewer = new AutonPreviewer(m_cyclePrims);
}
void Robot::InitializeDriveteamFeedback()
{
    m_field = DragonField::GetInstance(); // TODO: move to drive team feedback
}

void Robot::UpdateDriveTeamFeedback()
{
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }
    if (m_field != nullptr && m_chassis != nullptr)
    {
        m_field->UpdateRobotPosition(m_chassis->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
    }
    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }
}

void Robot::InitializeDataTracing()
{
    DataTrace::GetInstance()->Connect();
}
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
