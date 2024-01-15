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
#include "chassis/swerve/SwerveModuleBuilder.h"

SwerveModuleBuilder::SwerveModuleBuilder() : m_moduleID(SwerveModule::ModuleID::LEFT_FRONT),
                                             m_driveMotor(nullptr),
                                             m_turnMotor(nullptr),
                                             m_canCoder(nullptr),
                                             m_turnControlData(nullptr),
                                             m_countsOnTurnEncoderPerDegreesOnAngleSensor(1.0),
                                             m_wheelDiameter(units::length::inch_t(4.0))
{
}

void SwerveModuleBuilder::SetModuleID(SwerveModule::ModuleID id)
{
    m_moduleID = id;
}

void SwerveModuleBuilder::SetMotors(IDragonMotorController &driveMotor,
                                    IDragonMotorController &turnMotor)
{
    m_driveMotor = &driveMotor;
    m_turnMotor = &turnMotor;
}
void SwerveModuleBuilder::SetCanCoder(DragonCanCoder &angleSensor)
{
    m_canCoder = &angleSensor;
}

void SwerveModuleBuilder::SetTurnControlData(ControlData &pid, double countsOnTurnEncoderPerDegreesOnAngleSensor)
{
    m_turnControlData = &pid;
    m_countsOnTurnEncoderPerDegreesOnAngleSensor = countsOnTurnEncoderPerDegreesOnAngleSensor;
}

void SwerveModuleBuilder::SetWheelDiameter(units::length::inch_t diameter)
{
    m_wheelDiameter = diameter;
}

bool SwerveModuleBuilder::IsValid() const
{
    return m_driveMotor != nullptr &&
           m_turnMotor != nullptr &&
           m_canCoder != nullptr &&
           m_turnControlData != nullptr &&
           m_wheelDiameter.to<double>() > 0.0;
}
SwerveModule *SwerveModuleBuilder::Commit()
{
    SwerveModule *module = nullptr;
    if (IsValid())
    {
        module = new SwerveModule(m_moduleID,
                                  m_driveMotor,
                                  m_turnMotor,
                                  m_canCoder,
                                  *m_turnControlData,
                                  m_countsOnTurnEncoderPerDegreesOnAngleSensor,
                                  m_wheelDiameter);
    }
    return module;
}