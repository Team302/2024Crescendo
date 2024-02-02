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
#include <memory>
#include <string>

#include <frc/Filesystem.h>

// Team 302 includes
#include <auton/drivePrimitives/DragonTrajectoryUtils.h>
#include <auton/drivePrimitives/ResetPosition.h>
#include <auton/PrimitiveParams.h>
#include <auton/drivePrimitives/IPrimitive.h>
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "utils/logging/Logger.h"
#include <DragonVision/DragonVision.h>

using namespace std;
using namespace frc;

ResetPosition::ResetPosition() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

void ResetPosition::Init(PrimitiveParams *params)
{

    m_trajectory = DragonTrajectoryUtils::GetTrajectory(params);

    m_chassis->ResetPose(m_trajectory.InitialPose());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position"), string("Auton Info: ResetPosX"), m_chassis->GetPose().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position"), string("Auton Info: ResetPosY"), m_chassis->GetPose().Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position"), string("Auton Info: InitialPoseX"), m_trajectory.InitialPose().X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position"), string("Auton Info: InitialPoseY"), m_trajectory.InitialPose().Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Reset Position"), string("Auton Info: InitialPoseOmega"), m_trajectory.InitialPose().Rotation().Degrees().to<double>());
}

void ResetPosition::Run()
{
}

bool ResetPosition::IsDone()
{
    return true;
}