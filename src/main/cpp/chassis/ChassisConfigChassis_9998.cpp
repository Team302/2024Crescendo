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

#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleAttributes.h"
#include "chassis/SwerveModuleConstants.h"
#include "chassis/ChassisConfigChassis_9998.h"
#include "chassis/ChassisConfigMgr.h"
#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"

using std::string;

void ChassisConfigChassis_9998::DefineMotorControllers()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineMotorControllers"), string("arrived"));

    /**
    DistanceAngleCalcStruc calcStruc;
    m_leftFrontDrive = new DragonTalonFX(string("LF_SWERVE"),
                                         RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                         1,
                                         calcStruc,
                                         IDragonMotorController::MOTOR_TYPE::FALCON500,
                                         string("Canivore"));

    m_leftFrontTurn = new DragonTalonFX(string("LF_SWERVE"),
                                        RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                        3,
                                        calcStruc,
                                        IDragonMotorController::MOTOR_TYPE::FALCON500,
                                        string("Canivore"));

    m_rightFrontDrive = new DragonTalonFX(string("RF_SWERVE"),
                                          RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                          0,
                                          calcStruc,
                                          IDragonMotorController::MOTOR_TYPE::FALCON500,
                                          string("Canivore"));

    m_rightFrontTurn = new DragonTalonFX(string("RF_SWERVE"),
                                         RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                         2,
                                         calcStruc,
                                         IDragonMotorController::MOTOR_TYPE::FALCON500,
                                         string("Canivore"));

    m_leftBackDrive = new DragonTalonFX(string("LB_SWERVE"),
                                        RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                        18,
                                        calcStruc,
                                        IDragonMotorController::MOTOR_TYPE::FALCON500,
                                        string("Canivore"));

    m_leftBackTurn = new DragonTalonFX(string("LB_SWERVE"),
                                       RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                       16,
                                       calcStruc,
                                       IDragonMotorController::MOTOR_TYPE::FALCON500,
                                       string("Canivore"));

    m_rightBackDrive = new DragonTalonFX(string("RB_SWERVE"),
                                         RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE,
                                         17,
                                         calcStruc,
                                         IDragonMotorController::MOTOR_TYPE::FALCON500,
                                         string("Canivore"));

    m_rightBackTurn = new DragonTalonFX(string("RB_SWERVE"),
                                        RobotElementNames::MOTOR_CONTROLLER_USAGE::SWERVE_TURN,
                                        19,
                                        calcStruc,
                                        IDragonMotorController::MOTOR_TYPE::FALCON500,
                                        string("Canivore"));
**/
    /**
    m_leftFrontDrive->SetCurrentLimits(true,
       units::current::ampere_t(25.0),
       true,
       units::current::ampere_t(25.0),
       units::current::ampere_t(30.0),
       units::time::second_t(0.02));
    m_leftFrontDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
          ctre::phoenix6::signals::NeutralModeValue::Brake,
          0.01, -1, 1);

              m_leftFrontTurn->SetCurrentLimits(true,
                                      units::current::ampere_t(25.0),
                                      true,
                                      units::current::ampere_t(25.0),
                                      units::current::ampere_t(30.0),
                                      units::time::second_t(0.02));
    m_leftFrontTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                         ctre::phoenix6::signals::NeutralModeValue::Brake,
                                         0.01, -1, 1);

                                             m_rightFrontDrive->SetCurrentLimits(true,
                                        units::current::ampere_t(25.0),
                                        true,
                                        units::current::ampere_t(25.0),
                                        units::current::ampere_t(30.0),
                                        units::time::second_t(0.02));
    m_rightFrontDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive,
                                           ctre::phoenix6::signals::NeutralModeValue::Brake,
                                           0.01, -1, 1);
    m_rightFrontTurn->SetCurrentLimits(true,
                                       units::current::ampere_t(25.0),
                                       true,
                                       units::current::ampere_t(25.0),
                                       units::current::ampere_t(30.0),
                                       units::time::second_t(0.02));
    m_rightFrontTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                          ctre::phoenix6::signals::NeutralModeValue::Brake,
                                          0.01, -1, 1);

    m_leftBackDrive->SetCurrentLimits(true,
                                      units::current::ampere_t(25.0),
                                      true,
                                      units::current::ampere_t(25.0),
                                      units::current::ampere_t(30.0),
                                      units::time::second_t(0.02));
    m_leftBackDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                         ctre::phoenix6::signals::NeutralModeValue::Brake,
                                         0.01, -1, 1);
    m_leftBackTurn->SetCurrentLimits(true,
                                     units::current::ampere_t(25.0),
                                     true,
                                     units::current::ampere_t(25.0),
                                     units::current::ampere_t(30.0),
                                     units::time::second_t(0.02));
    m_leftBackTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                        ctre::phoenix6::signals::NeutralModeValue::Brake,
                                        0.01, -1, 1);
    m_rightBackDrive->SetCurrentLimits(true,
                                       units::current::ampere_t(25.0),
                                       true,
                                       units::current::ampere_t(25.0),
                                       units::current::ampere_t(30.0),
                                       units::time::second_t(0.02));
    m_rightBackDrive->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive,
                                          ctre::phoenix6::signals::NeutralModeValue::Brake,
                                          0.01, -1, 1);
    m_rightBackTurn->SetCurrentLimits(true,
                                      units::current::ampere_t(25.0),
                                      true,
                                      units::current::ampere_t(25.0),
                                      units::current::ampere_t(30.0),
                                      units::time::second_t(0.02));
    m_rightBackTurn->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                         ctre::phoenix6::signals::NeutralModeValue::Brake,
                                         0.01, -1, 1);

    **/

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineMotorControllers"), string("end"));
}
void ChassisConfigChassis_9998::DefineCANSensors()
{
    /**
    m_pigeon = new DragonPigeon2(0,
                                 string("Canivore"),
                                 RobotElementNames::PIGEON_USAGE::PIGEON_ROBOT_CENTER,
                                 units::angle::degree_t(0.0),
                                 units::angle::degree_t(0.0),
                                 units::angle::degree_t(0.0));
**/
    /**
    m_leftFrontCoder = new DragonCanCoder(string("RB_SWERVE"),
                                       RobotElementNames::CANCODER_USAGE::BACK_RIGHT_SWERVE,
                                       19,
                                       string("Canivore"),
                                       156.7092,
                                       false);

    m_rightFrontCoder = new DragonCanCoder(string("RF_SWERVE"),
                                        RobotElementNames::CANCODER_USAGE::FRONT_RIGHT_SWERVE,
                                        2,
                                        string("Canivore"),
                                        -51.5923,
                                        false);

    m_leftBackCoder = new DragonCanCoder(string("LB_SWERVE"),
                                      RobotElementNames::CANCODER_USAGE::BACK_LEFT_SWERVE,
                                      19,
                                      string("Canivore"),
                                      43.945,
                                      false);

    m_rightBackCoder = new DragonCanCoder(string("LF_SWERVE"),
                                       RobotElementNames::CANCODER_USAGE::FRONT_LEFT_SWERVE,
                                       3,
                                       string("Canivore"),
                                       -105.3812,
                                       false);
                                       **/
}
void ChassisConfigChassis_9998::DefineChassis()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("arrived"));
    m_leftFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                         1, false,
                                         3, false,
                                         3, -105.3812);
    m_leftBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                        18, false,
                                        16, false,
                                        16, 43.945);
    m_rightFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                          0, true,
                                          2, false,
                                          2, -51.5923);
    m_rightBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                         17, true,
                                         19, false,
                                         19, 156.7092);
    /**
    m_leftFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                     SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                     m_leftFrontDrive,
                                     m_leftFrontTurn,
                                     m_leftFrontCoder);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("module 1"));
    m_rightFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                      SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                      m_rightFrontDrive,
                                      m_rightFrontTurn,
                                      m_rightFrontCoder);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("module 2"));
    m_leftBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_BACK,
                                    SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                    m_leftBackDrive,
                                    m_leftBackTurn,
                                    m_leftBackCoder);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("module 3"));
    m_rightBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                     SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                     m_rightBackDrive,
                                     m_rightBackTurn,
                                     m_rightBackCoder);
                                        **/
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("module 4"));
    /**/
    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon,
                                  units::length::inch_t(22.75),
                                  units::length::inch_t(22.75),
                                  string("SwerveChassis"));
    /**/
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("end"));
}
