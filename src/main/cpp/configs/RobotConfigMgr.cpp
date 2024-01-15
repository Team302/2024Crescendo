
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

#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigExample.h"

RobotConfigMgr *RobotConfigMgr::m_instance = nullptr;
RobotConfigMgr *RobotConfigMgr::GetInstance()
{
    if (RobotConfigMgr::m_instance == nullptr)
    {
        RobotConfigMgr::m_instance = new RobotConfigMgr();
    }
    return RobotConfigMgr::m_instance;
}

RobotConfigMgr::RobotConfigMgr() : m_config(nullptr)
{
}

void RobotConfigMgr::InitRobot(RobotIdentifier id)
{
    switch (id)
    {
    case RobotIdentifier::EXAMPLE:
        m_config = new RobotConfigExample();
        break;

    case RobotIdentifier::COMP_2023:
        break;

    case RobotIdentifier::PRACTICE_2023:
        break;

    case RobotIdentifier::CONNECT4_2021:
        break;

    case RobotIdentifier::CONNECT4_2022:
        break;

    case RobotIdentifier::THING1:
        break;
    case RobotIdentifier::THING2:
        break;
    case RobotIdentifier::THING3:
        break;
    case RobotIdentifier::THING4:
        break;

    default:
        break;
    }

    if (m_config != nullptr)
    {
        m_config->BuildRobot();
    }
}
