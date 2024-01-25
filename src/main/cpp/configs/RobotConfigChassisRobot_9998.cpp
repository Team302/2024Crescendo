//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// This file was manually  generated
// Generated on Monday, January 22, 2024

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigChassisRobot_9998.h"
#include "chassis/swerve/SwerveModule.h"
#include "chassis/swerve/SwerveModuleAttributes.h"
#include "chassis/swerve/SwerveModuleConstants.h"

using std::string;

void RobotConfigChassisRobot_9998::DefineMotorControllers()
{
       DistanceAngleCalcStruc calcStruc;
       m_frontLeftDrive = new DragonTalonFX(string("LF_SWERVE"),
                                            RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                            1,
                                            calcStruc,
                                            IDragonMotorController::MOTOR_TYPE::FALCON500,
                                            string("Canivore"));
       m_frontLeftDrive->SetCurrentLimits(true,
                                          units::current::ampere_t(25.0),
                                          true,
                                          units::current::ampere_t(25.0),
                                          units::current::ampere_t(30.0),
                                          units::time::second_t(0.02));
       m_frontLeftDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                             ctre::phoenix6::signals::NeutralModeValue::Brake,
                                             0.01, -1, 1);

       m_frontLeftTurn = new DragonTalonFX(string("LF_SWERVE"),
                                           RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                           3,
                                           calcStruc,
                                           IDragonMotorController::MOTOR_TYPE::FALCON500,
                                           string("Canivore"));
       m_frontLeftTurn->SetCurrentLimits(true,
                                         units::current::ampere_t(25.0),
                                         true,
                                         units::current::ampere_t(25.0),
                                         units::current::ampere_t(30.0),
                                         units::time::second_t(0.02));
       m_frontLeftTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                            ctre::phoenix6::signals::NeutralModeValue::Brake,
                                            0.01, -1, 1);

       m_frontRightDrive = new DragonTalonFX(string("RF_SWERVE"),
                                             RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                             0,
                                             calcStruc,
                                             IDragonMotorController::MOTOR_TYPE::FALCON500,
                                             string("Canivore"));
       m_frontRightDrive->SetCurrentLimits(true,
                                           units::current::ampere_t(25.0),
                                           true,
                                           units::current::ampere_t(25.0),
                                           units::current::ampere_t(30.0),
                                           units::time::second_t(0.02));
       m_frontRightDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive,
                                              ctre::phoenix6::signals::NeutralModeValue::Brake,
                                              0.01, -1, 1);

       m_frontRightTurn = new DragonTalonFX(string("RF_SWERVE"),
                                            RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                            2,
                                            calcStruc,
                                            IDragonMotorController::MOTOR_TYPE::FALCON500,
                                            string("Canivore"));
       m_frontRightTurn->SetCurrentLimits(true,
                                          units::current::ampere_t(25.0),
                                          true,
                                          units::current::ampere_t(25.0),
                                          units::current::ampere_t(30.0),
                                          units::time::second_t(0.02));
       m_frontRightTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                             ctre::phoenix6::signals::NeutralModeValue::Brake,
                                             0.01, -1, 1);

       m_backLeftDrive = new DragonTalonFX(string("LB_SWERVE"),
                                           RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                           18,
                                           calcStruc,
                                           IDragonMotorController::MOTOR_TYPE::FALCON500,
                                           string("Canivore"));
       m_backLeftDrive->SetCurrentLimits(true,
                                         units::current::ampere_t(25.0),
                                         true,
                                         units::current::ampere_t(25.0),
                                         units::current::ampere_t(30.0),
                                         units::time::second_t(0.02));
       m_backLeftDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                            ctre::phoenix6::signals::NeutralModeValue::Brake,
                                            0.01, -1, 1);

       m_backLeftTurn = new DragonTalonFX(string("LB_SWERVE"),
                                          RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                          16,
                                          calcStruc,
                                          IDragonMotorController::MOTOR_TYPE::FALCON500,
                                          string("Canivore"));
       m_backLeftTurn->SetCurrentLimits(true,
                                        units::current::ampere_t(25.0),
                                        true,
                                        units::current::ampere_t(25.0),
                                        units::current::ampere_t(30.0),
                                        units::time::second_t(0.02));
       m_backLeftTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                           ctre::phoenix6::signals::NeutralModeValue::Brake,
                                           0.01, -1, 1);

       m_backRightDrive = new DragonTalonFX(string("RB_SWERVE"),
                                            RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                            17,
                                            calcStruc,
                                            IDragonMotorController::MOTOR_TYPE::FALCON500,
                                            string("Canivore"));
       m_backRightDrive->SetCurrentLimits(true,
                                          units::current::ampere_t(25.0),
                                          true,
                                          units::current::ampere_t(25.0),
                                          units::current::ampere_t(30.0),
                                          units::time::second_t(0.02));
       m_backRightDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive,
                                             ctre::phoenix6::signals::NeutralModeValue::Brake,
                                             0.01, -1, 1);

       m_backRightTurn = new DragonTalonFX(string("RB_SWERVE"),
                                           RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                           19,
                                           calcStruc,
                                           IDragonMotorController::MOTOR_TYPE::FALCON500,
                                           string("Canivore"));
       m_backRightTurn->SetCurrentLimits(true,
                                         units::current::ampere_t(25.0),
                                         true,
                                         units::current::ampere_t(25.0),
                                         units::current::ampere_t(30.0),
                                         units::time::second_t(0.02));
       m_backRightTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                            ctre::phoenix6::signals::NeutralModeValue::Brake,
                                            0.01, -1, 1);
}
void RobotConfigChassisRobot_9998::DefineCANSensors()
{
       m_pigeon = new DragonPigeon2(0,
                                    string("Canivore"),
                                    RobotElementNames::PIGEON_USAGE::PIGEON_ROBOT_CENTER,
                                    units::angle::degree_t(0.0),
                                    units::angle::degree_t(0.0),
                                    units::angle::degree_t(0.0));

       m_frontLeftCC = new DragonCanCoder(string("RB_SWERVE"),
                                          RobotElementNames::CANCODER_USAGE::BACK_RIGHT_SWERVE,
                                          19,
                                          string("Canivore"),
                                          156.7092,
                                          false);

       m_frontRightCC = new DragonCanCoder(string("RF_SWERVE"),
                                           RobotElementNames::CANCODER_USAGE::FRONT_RIGHT_SWERVE,
                                           2,
                                           string("Canivore"),
                                           -51.5923,
                                           false);

       m_backLeftCC = new DragonCanCoder(string("LB_SWERVE"),
                                         RobotElementNames::CANCODER_USAGE::BACK_LEFT_SWERVE,
                                         19,
                                         string("Canivore"),
                                         43.945,
                                         false);

       m_backRightCC = new DragonCanCoder(string("LF_SWERVE"),
                                          RobotElementNames::CANCODER_USAGE::FRONT_LEFT_SWERVE,
                                          3,
                                          string("Canivore"),
                                          -105.3812,
                                          false);
}
void RobotConfigChassisRobot_9998::DefineChassis()
{
       m_frontLeftSM = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                        SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                        m_frontLeftDrive,
                                        m_frontLeftTurn,
                                        m_frontLeftCC);
       m_frontRightSM = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                         SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                         m_frontRightDrive,
                                         m_frontRightTurn,
                                         m_frontRightCC);
       m_backLeftSM = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_BACK,
                                       SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                       m_backLeftDrive,
                                       m_backLeftTurn,
                                       m_backLeftCC);
       m_backRightSM = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                        SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                        m_backRightDrive,
                                        m_backRightTurn,
                                        m_backRightCC);

       m_swerveChassis = new SwerveChassis(m_frontLeftSM,
                                           m_frontRightSM,
                                           m_backLeftSM,
                                           m_backRightSM,
                                           m_pigeon,
                                           units::length::inch_t(22.75),
                                           units::length::inch_t(22.75),
                                           string("SwerveChassis"));
}

SwerveChassis *RobotConfigChassisRobot_9998::GetSwerveChassis() const
{
       return m_swerveChassis;
}
