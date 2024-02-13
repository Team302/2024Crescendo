
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
#include "chassis/IChassis.h"

#include "configs/RobotElementNames.h"

#include "hw/interfaces/IDragonMotorController.h"
#include "hw/interfaces/IDragonPigeon.h"
#include "hw/DragonAnalogInput.h"
#include "hw/DragonCanCoder.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonServo.h"
#include "hw/DragonSolenoid.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/MechanismTypes.h"

class RobotConfig
{
public:
    RobotConfig();
    ~RobotConfig();

    void BuildRobot();

    virtual SwerveChassis *GetSwerveChassis() const;
    virtual IChassis *GetIChassis() const;

    virtual IDragonMotorController *GetMotorController(RobotElementNames::MOTOR_CONTROLLER_USAGE usage);
    virtual DragonSolenoid *GetSolenoid(RobotElementNames::SOLENOID_USAGE usage);
    virtual DragonServo *GetServo(RobotElementNames::SERVO_USAGE usage);

    virtual DragonCanCoder *GetCanCoder(RobotElementNames::CANCODER_USAGE usage);
    virtual IDragonPigeon *GetPigeon(RobotElementNames::PIGEON_USAGE usage);

    virtual StateMgr *GetMechanism(MechanismTypes::MECHANISM_TYPE mechType) = 0;
    // TODO:  add methods to access mechanisms and hardware as necessary

protected:
    virtual void DefineBuiltInControlItems();

    // actuators
    virtual void DefineMotorControllers();
    virtual void DefineSolenoids();
    virtual void DefineServos();

    // sensors
    virtual void DefineCANSensors();
    virtual void DefineDigitalInputSensors();
    virtual void DefineAnalogInputSensors();
    virtual void DefineI2CSensors();
    virtual void DefineVisionSensors();
    virtual void DefinePWMIO();

    // mechanisms
    virtual void DefineChassis();
    virtual void DefineMechanisms();

    virtual void DefineLEDs();
};
