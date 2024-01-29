
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

#pragma once

#include "chassis/SwerveChassis.h"
#include "chassis/SwerveModule.h"

#include "hw/DragonCanCoder.h"
#include "hw/DragonPigeon2.h"
#include "hw/DragonTalonFX.h"

class ChassisConfig
{
public:
    enum SWERVE_MODULE
    {
        LEFT_FRONT,
        LEFT_BACK,
        RIGHT_FRONT,
        RIGHT_BACK
    };

    enum MOTOR_TYPE
    {
        TURN,
        DRIVE
    };

    ChassisConfig();
    ~ChassisConfig();

    void BuildChassis();

    SwerveChassis *GetSwerveChassis() const { return m_chassis; }
    SwerveModule *GetSwerveModule(SWERVE_MODULE module) const;

    DragonTalonFX *GetMotorController(SWERVE_MODULE module,
                                      MOTOR_TYPE type);

    DragonCanCoder *GetCanCoder(SWERVE_MODULE module);
    DragonPigeon2 *GetPigeon() const { return m_pigeon; }

protected:
    // actuators
    virtual void DefineMotorControllers();

    // sensors
    virtual void DefineCANSensors();

    // mechanisms
    virtual void DefineChassis();

    DragonCanCoder *m_leftFrontCoder;
    DragonCanCoder *m_leftBackCoder;
    DragonCanCoder *m_rightFrontCoder;
    DragonCanCoder *m_rightBackCoder;

    DragonTalonFX *m_leftBackDrive;
    DragonTalonFX *m_leftBackTurn;
    DragonTalonFX *m_leftFrontDrive;
    DragonTalonFX *m_leftFrontTurn;
    DragonTalonFX *m_rightBackDrive;
    DragonTalonFX *m_rightBackTurn;
    DragonTalonFX *m_rightFrontDrive;
    DragonTalonFX *m_rightFrontTurn;

    DragonPigeon2 *m_pigeon;

    SwerveModule *m_leftBackModule;
    SwerveModule *m_leftFrontModule;
    SwerveModule *m_rightBackModule;
    SwerveModule *m_rightFrontModule;

    SwerveChassis *m_chassis;
};
