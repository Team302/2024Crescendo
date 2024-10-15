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

// C++
#include <string>

// FRC Includes

// 302 Includes
#include <auton/drivePrimitives/VisionDrivePrimitive.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"
#include <iostream>

VisionDrivePrimitive::VisionDrivePrimitive() : IPrimitive(),
                                               m_chassis(nullptr),
                                               m_headingOption(ChassisOptionEnums::HeadingOption::MAINTAIN),
                                               m_ntName("VisionDrivePrimitive"),
                                               m_timer(new frc::Timer()),
                                               m_timeout(0.0)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

void VisionDrivePrimitive::Init(PrimitiveParams *params)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtInit", true);

    // m_pipelineMode = params->GetPipeline();
    m_timeout = params->GetTime();

    m_timer->Reset();
    m_timer->Start();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "m_pipelineMode", m_pipelineMode);
}

void VisionDrivePrimitive::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtRun", true);

    if (m_chassis != nullptr)
    {
        ChassisMovement moveInfo;

        moveInfo.headingOption = m_headingOption;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "driveOption", moveInfo.driveOption);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "headingOption", moveInfo.headingOption);

        m_chassis->Drive(moveInfo);
    }
}

bool VisionDrivePrimitive::IsDone()
{
    return false;
}
