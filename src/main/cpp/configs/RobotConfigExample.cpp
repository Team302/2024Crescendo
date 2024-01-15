
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
#include <string>

#include "configs/RobotConfigExample.h"
#include "configs/usages/CanSensorUsage.h"
#include "configs/usages/MotorControllerUsage.h"
#include "hw/DragonCanCoder.h"
#include "hw/DragonTalonFX.h"
#include "hw/DistanceAngleCalcStruc.h"
#include "hw/interfaces/IDragonMotorController.h"
#include "mechanisms/example/decoratormods/Example.h"
#include "mechanisms/example/generated/ExampleGen.h"

constexpr char canBusName[] = "Canivore";

using std::string;

void RobotConfigExample::DefineMotorControllers()
{
    DistanceAngleCalcStruc distCalc;
    IDragonMotorController::MOTOR_TYPE mtype = IDragonMotorController::MOTOR_TYPE::FALCON500;

    m_motor1 = new DragonTalonFX(string("ExampleMech_Motor1"),
                                 MotorControllerUsage::MOTOR_CONTROLLER_USAGE::EXAMPLE_MOTOR1,
                                 1,
                                 distCalc,
                                 mtype,
                                 string(canBusName));
    m_motor1->SetCurrentLimits(true,
                               units::current::ampere_t(25.0),
                               true,
                               units::current::ampere_t(25.0),
                               units::current::ampere_t(30.0),
                               units::time::second_t(0.02));
    m_motor1->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                  ctre::phoenix6::signals::NeutralModeValue::Brake,
                                  0.01, -1, 1);

    m_motor2 = new DragonTalonFX(string("ExampleMech_Motor2"),
                                 MotorControllerUsage::MOTOR_CONTROLLER_USAGE::EXAMPLE_MOTOR2,
                                 2,
                                 distCalc,
                                 mtype,
                                 string(canBusName));
    m_motor2->SetCurrentLimits(true,
                               units::current::ampere_t(25.0),
                               true,
                               units::current::ampere_t(25.0),
                               units::current::ampere_t(30.0),
                               units::time::second_t(0.02));
    m_motor2->ConfigMotorSettings(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive,
                                  ctre::phoenix6::signals::NeutralModeValue::Brake,
                                  0.01, -1, 1);
    m_motor2->FuseCancoder(*m_cancoder, 1.0, 12.8);
}

void RobotConfigExample::DefineSolenoids()
{
}

void RobotConfigExample::DefineMechanisms()
{
    // TODO:  utilize the motors, solenoids, etc. to create the mechanisms
    auto genmech = new ExampleGen(string("Example.xml"), string("ExampleMech"));
    m_example = new Example(genmech);
    m_example->AddMotor(*m_motor1);
    m_example->AddMotor(*m_motor2);
}

void RobotConfigExample::DefineCANSensors()
{
    m_cancoder = new DragonCanCoder(string("ExampleMech_Motor1"),
                                    CanSensorUsage::CANSENSOR_USAGE::EXAMPLE_CANCODER,
                                    0,
                                    string(canBusName),
                                    35.0,
                                    false);
}
