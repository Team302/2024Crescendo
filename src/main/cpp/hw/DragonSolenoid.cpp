
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

#include <memory>
#include <string>

#include <frc/Solenoid.h>
#include "hw/DragonSolenoid.h"
#include <configs/RobotElementNames.h>
#include "utils/logging/Logger.h"
#include <frc/Compressor.h>

using namespace frc;
using namespace std;

DragonSolenoid::DragonSolenoid(
    string networkTableName,
    RobotElementNames::SOLENOID_USAGE usage,
    int pcmID,
    PneumaticsModuleType pcmType,
    int channel,
    bool reversed)
{
    InitSingle(networkTableName, usage, pcmID, pcmType, channel, reversed);
}

DragonSolenoid::DragonSolenoid(
    string networkTableName,
    RobotElementNames::SOLENOID_USAGE usage,
    int pcmID,
    PneumaticsModuleType pcmType,
    int forwardChannel,
    int reverseChannel,
    bool reversed)
{
    InitDouble(networkTableName, usage, pcmID, pcmType, forwardChannel, reverseChannel, reversed);
}

void DragonSolenoid::Set(bool on)
{
    if (m_solenoid != nullptr)
    {
        bool val = (m_reversed) ? !on : on;
        m_solenoid->Set(val);
    }
    else if (m_doubleSolenoid != nullptr)
    {
        DoubleSolenoid::Value val = on ? DoubleSolenoid::Value::kForward : DoubleSolenoid::Value::kReverse;
        if (m_reversed)
        {
            if (val == DoubleSolenoid::Value::kForward)
            {
                val = DoubleSolenoid::Value::kReverse;
            }
            else
            {
                val = DoubleSolenoid::Value::kForward;
            }
        }
        m_doubleSolenoid->Set(val);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("DragonSolenoid::Set"), string("solenoid ptr is nullptr"));
    }
}

void DragonSolenoid::Set(
    DoubleSolenoid::Value in)
{
    if (m_doubleSolenoid != nullptr)
    {
        DoubleSolenoid::Value val = in;
        if (m_reversed)
        {
            if (val == DoubleSolenoid::Value::kForward)
            {
                val = DoubleSolenoid::Value::kReverse;
            }
            else
            {
                val = DoubleSolenoid::Value::kForward;
            }
        }
        m_doubleSolenoid->Set(val);
    }
    else if (m_solenoid != nullptr)
    {
        auto on = in == DoubleSolenoid::Value::kForward;
        auto val = (m_reversed) ? !on : on;
        m_solenoid->Set(val);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("DragonSolenoid::Set"), string("solenoid ptr is nullptr"));
    }
}

void DragonSolenoid::InitSingle(
    string networkTableName,
    RobotElementNames::SOLENOID_USAGE usage,
    int pcmID,
    frc::PneumaticsModuleType pcmType,
    int channel,
    bool reversed)
{
    m_networkTableName = networkTableName;
    m_usage = usage;
    m_solenoid = new Solenoid(pcmID, pcmType, channel);
    m_doubleSolenoid = nullptr;
    m_reversed = reversed;
}

void DragonSolenoid::InitDouble(
    string networkTableName,
    RobotElementNames::SOLENOID_USAGE usage,
    int pcmID,
    frc::PneumaticsModuleType pcmType,
    int forwardChannel,
    int reverseChannel,
    bool reversed)
{
    m_networkTableName = networkTableName;
    m_usage = usage;
    m_solenoid = nullptr;
    m_doubleSolenoid = new DoubleSolenoid(pcmID, pcmType, forwardChannel, reverseChannel);
    m_reversed = reversed;
}

bool DragonSolenoid::Get() const
{
    bool val = false;
    if (m_solenoid != nullptr)
    {
        val = (m_reversed) ? !m_solenoid->Get() : m_solenoid->Get();
    }
    else if (m_doubleSolenoid != nullptr)
    {
        val = m_doubleSolenoid->Get() == DoubleSolenoid::Value::kForward;
        val = (m_reversed) ? !val : val;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("DragonSolenoid::Get"), string("solenoid ptr is nullptr"));
    }
    return val;
}
bool DragonSolenoid::IsDisabled() const
{
    bool val = false;
    if (m_solenoid != nullptr)
    {
        val = m_solenoid->IsDisabled();
    }
    else if (m_doubleSolenoid != nullptr)
    {
        val = m_doubleSolenoid->IsFwdSolenoidDisabled();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("DragonSolenoid::IsDisabled"), string("solenoid ptr is nullptr"));
    }
    return val;
}

RobotElementNames::SOLENOID_USAGE DragonSolenoid::GetType() const
{
    return m_usage;
}
