
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

#include "hw/MotorData.h"
#include "hw/interfaces/IDragonMotorController.h"

MotorData *MotorData::m_instance = nullptr;
MotorData *MotorData::GetInstance()
{
    if (MotorData::m_instance == nullptr)
    {
        MotorData::m_instance = new MotorData();
    }
    return MotorData::m_instance;
}
int MotorData::getStallCurrent(IDragonMotorController::MOTOR_TYPE motorType) const
{
    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return stallCurrentValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return stallCurrentValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return stallCurrentValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return stallCurrentValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return stallCurrentValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return stallCurrentValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return stallCurrentValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return stallCurrentValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return stallCurrentValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return stallCurrentValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return stallCurrentValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return stallCurrentValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return stallCurrentValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return stallCurrentValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return stallCurrentValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return stallCurrentValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return stallCurrentValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];
    case IDragonMotorController::KRAKEN60:
        return stallCurrentValues[IDragonMotorController::KRAKEN60];
    case IDragonMotorController::VORTEX:
        return stallCurrentValues[IDragonMotorController::VORTEX];

    default:
        return 0;
    }
}
double MotorData::getFreeCurrent(IDragonMotorController::MOTOR_TYPE motorType) const
{

    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return freeCurrentValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return freeCurrentValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return freeCurrentValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return freeCurrentValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return freeCurrentValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return freeCurrentValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return freeCurrentValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return freeCurrentValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return freeCurrentValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return freeCurrentValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return freeCurrentValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return freeCurrentValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return freeCurrentValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return freeCurrentValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return freeCurrentValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return freeCurrentValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return freeCurrentValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];
    case IDragonMotorController::KRAKEN60:
        return freeCurrentValues[IDragonMotorController::KRAKEN60];
    case IDragonMotorController::VORTEX:
        return freeCurrentValues[IDragonMotorController::VORTEX];
    default:
        return 0;
    }
}
int MotorData::getFreeSpeed(IDragonMotorController::MOTOR_TYPE motorType) const
{

    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return freeSpeedValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return freeSpeedValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return freeSpeedValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return freeSpeedValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return freeSpeedValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return freeSpeedValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return freeSpeedValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return freeSpeedValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return freeSpeedValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return freeSpeedValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return freeSpeedValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return freeSpeedValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return freeSpeedValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return freeSpeedValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return freeSpeedValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return freeSpeedValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return freeSpeedValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];
    case IDragonMotorController::KRAKEN60:
        return freeSpeedValues[IDragonMotorController::KRAKEN60];
    case IDragonMotorController::VORTEX:
        return freeSpeedValues[IDragonMotorController::VORTEX];
    default:
        return 0;
    }
}
int MotorData::getMaximumPower(IDragonMotorController::MOTOR_TYPE motorType) const
{

    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return maximumPowerValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return maximumPowerValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return maximumPowerValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return maximumPowerValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return maximumPowerValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return maximumPowerValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return maximumPowerValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return maximumPowerValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return maximumPowerValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return maximumPowerValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return maximumPowerValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return maximumPowerValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return maximumPowerValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return maximumPowerValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return maximumPowerValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return maximumPowerValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return maximumPowerValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];
    case IDragonMotorController::KRAKEN60:
        return maximumPowerValues[IDragonMotorController::KRAKEN60];
    case IDragonMotorController::VORTEX:
        return maximumPowerValues[IDragonMotorController::VORTEX];
    default:
        return 0;
    }
}
double MotorData::getStallTorque(IDragonMotorController::MOTOR_TYPE motorType) const
{
    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return stallTorqueValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return stallTorqueValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return stallTorqueValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return stallTorqueValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return stallTorqueValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return stallTorqueValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return stallTorqueValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return stallTorqueValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return stallTorqueValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return stallTorqueValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return stallTorqueValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return stallTorqueValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return stallTorqueValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return stallTorqueValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return stallTorqueValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return stallTorqueValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return stallTorqueValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];
    case IDragonMotorController::KRAKEN60:
        return stallTorqueValues[IDragonMotorController::KRAKEN60];
    case IDragonMotorController::VORTEX:
        return stallTorqueValues[IDragonMotorController::VORTEX];
    default:
        return 0;
    }
}

bool MotorData::checkIfStall(std::shared_ptr<IDragonMotorController> motor)
{

    auto motorType = motor.get()->GetMotorType();
    return motor.get()->GetCurrent() >= getStallCurrent(motorType) * 0.90;
}