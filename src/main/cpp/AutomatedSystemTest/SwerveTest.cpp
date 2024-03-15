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
#include "SwerveTest.h"
#include "chassis\SwerveChassis.h"
#include "hw\factories\PDPFactory.h"
#include "chassis\ChassisConfigMgr.h"
SwerveTest::SwerveTest()
{
    auto swerveChassisptr = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    bool XBackwardFinished = false;
    bool YBackwardFinished = false;
    bool XFrowardFinished = false;
    bool YForwardFinished = false;
}
SwerveTest *SwerveTest::m_swerveTest = nullptr;
SwerveTest *SwerveTest::GetInstance()
{
    if (m_swerveTest == nullptr)
    {
        m_swerveTest = new SwerveTest();
    }
    return m_swerveTest;
}
bool SwerveTest::RunSwerveTest()
{
    return false;
}
void SwerveTest::XBackwardTest()
{
    auto swerveChassisptr = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    double voltage = PDPFactory::GetFactory()->GetPDP()->GetVoltage();
    double totalEnergy = PDPFactory::GetFactory()->GetPDP()->GetTotalEnergy();
    double totalCurrents = PDPFactory::GetFactory()->GetPDP()->GetTotalCurrent();
    double channelcurrents = PDPFactory::GetFactory()->GetPDP()->GetCurrent(1);
    double temperature = PDPFactory::GetFactory()->GetPDP()->GetTemperature();

    if (swerveChassisptr != nullptr)
    {
        auto maxSpeed = swerveChassisptr->GetMaxSpeed();
        ChassisMovement moveInfo;
        timer++;
        if (timer < 500)
        {
            moveInfo.chassisSpeeds.vx = -.5 * maxSpeed;
        }
        else if (timer > 499)
        {
            moveInfo.chassisSpeeds.vx = 0 * maxSpeed;
            XBackwardFinished = true;
        }
        swerveChassisptr->Drive(moveInfo);
        loggerData = {0.0, 0.0, 0.0, 0.0};
    }
}
void SwerveTest::XForwardTest()
{
    auto swerveChassisptr = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    if (swerveChassisptr != nullptr)
    {
        auto maxSpeed = swerveChassisptr->GetMaxSpeed();
        ChassisMovement moveInfo;
        timer++;
        if (timer < 500)
        {
            moveInfo.chassisSpeeds.vx = .5 * maxSpeed;
        }
        else if (timer > 499)
        {
            moveInfo.chassisSpeeds.vx = 0 * maxSpeed;
            XForwardFinished = true;
        }
        swerveChassisptr->Drive(moveInfo);
    }
}
void SwerveTest::YBackwardTest()
{
    auto swerveChassisptr = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    if (swerveChassisptr != nullptr)
    {
        auto maxSpeed = swerveChassisptr->GetMaxSpeed();
        ChassisMovement moveInfo;
        timer++;
        if (timer < 500)
        {
            moveInfo.chassisSpeeds.vy = -.5 * maxSpeed;
        }
        else if (timer > 499)
        {
            moveInfo.chassisSpeeds.vy = 0 * maxSpeed;
            YBackwardFinished = true;
        }
        swerveChassisptr->Drive(moveInfo);
    }
}
void SwerveTest::YForwardTest()
{
    auto swerveChassisptr = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    if (swerveChassisptr != nullptr)
    {
        auto maxSpeed = swerveChassisptr->GetMaxSpeed();
        ChassisMovement moveInfo;
        timer++;
        if (timer < 500)
        {
            moveInfo.chassisSpeeds.vy = .5 * maxSpeed;
        }
        else if (timer > 499)
        {
            moveInfo.chassisSpeeds.vy = 0 * maxSpeed;
            YForwardFinished = true;
        }
        swerveChassisptr->Drive(moveInfo);
    }
}