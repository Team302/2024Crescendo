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
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <DragonVision/LimelightUsages.h>
#include "utils/logging/Logger.h"

// Third Party Includes

LimelightUsages *LimelightUsages::m_instance = nullptr;
LimelightUsages *LimelightUsages::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new LimelightUsages();
    }
    return m_instance;
}

LimelightUsages::LimelightUsages()
{
    m_usageMap["MAINLIMELIGHT"] = LIMELIGHT_USAGE::PRIMARY;
    m_usageMap["SECONDARYLIMELIGHT"] = LIMELIGHT_USAGE::SECONDARY;
}

LimelightUsages::~LimelightUsages()
{
    m_usageMap.clear();
}

LimelightUsages::LIMELIGHT_USAGE LimelightUsages::GetUsage(std::string usageString)
{
    auto it = m_usageMap.find(usageString);
    if (it != m_usageMap.end())
    {
        return it->second;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("LimelightUsages::GetUsage"), std::string("unknown usage"), usageString);
    return LimelightUsages::LIMELIGHT_USAGE::UNKNOWN_USAGE;
}
