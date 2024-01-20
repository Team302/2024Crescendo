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

// FRC Includes

// Team 302 Includes
#include "chassis/swerve/SwerveModuleConstants.h"

// Third Party Includes

SwerveModuleAttributes SwerveModuleConstants::GetSwerveModuleAttrs(ModuleType type)
{
    SwerveModuleAttributes attr;
    attr.wheelDiameter = units::length::inch_t(4.0);

    switch (type)
    {
    case SDS_MK4_L1:
        attr.driveGearRatio = 8.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 12.8;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L2:
        attr.driveGearRatio = 6.75;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 12.8;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L3:
        attr.driveGearRatio = 6.12;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 12.8;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L4:
        attr.driveGearRatio = 5.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 12.8;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L1_COLSON:
        attr.driveGearRatio = 6.75;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L2_COLSON:
        attr.driveGearRatio = 6.12;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L3_COLSON:
        attr.driveGearRatio = 5.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4_L4_COLSON:
        attr.driveGearRatio = 8.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L1:
        attr.driveGearRatio = 8.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L2:
        attr.driveGearRatio = 6.75;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L3:
        attr.driveGearRatio = 6.12;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L1_COLSON:
        attr.driveGearRatio = 8.14;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L2_COLSON:
        attr.driveGearRatio = 6.75;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    case SDS_MK4I_L3_COLSON:
        attr.driveGearRatio = 6.12;
        attr.driveControl.SetMode(ControlModes::CONTROL_TYPE::VELOCITY_RPS);
        attr.driveControl.SetP(0.1);
        attr.driveControl.SetI(0.0);
        attr.driveControl.SetD(0.0);
        attr.driveControl.SetF(0.0);

        attr.angleGearRatio = 150.0 / 7.0;
        attr.angleControl.SetMode(ControlModes::CONTROL_TYPE::TRAPEZOID);
        attr.angleControl.SetP(30.0);
        attr.angleControl.SetI(0.0);
        attr.angleControl.SetD(0.2);
        attr.angleControl.SetF(0.0);
        attr.angleControl.SetCruiseVelocity(1.0);
        attr.angleControl.SetMaxAcceleration(1.0);

        attr.sensorToMechanismRatio = 1.0;
        attr.rotorToSensorRatio = 12.8;
        break;

    default:
        break;
    }

    return attr;
}