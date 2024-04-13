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

// Team 302 includes
#include "auton/drivePrimitives/IPrimitive.h"
#include "auton/drivePrimitives/ResetPositionPathPlanner.h"
#include "auton/PrimitiveParams.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "DragonVision/DragonVision.h"
#include "utils/logging/Logger.h"
#include "utils/FMSData.h"

// Third Party Includes
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace std;
using namespace frc;
using namespace pathplanner;

ResetPositionPathPlanner::ResetPositionPathPlanner() : IPrimitive()
{
}

void ResetPositionPathPlanner::Init(PrimitiveParams *param)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();
        auto position = vision->GetRobotPosition();
        if (position)
        {
            auto pose = position.value().estimatedPose.ToPose2d();

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("vision x position"), pose.X().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("vision y position"), pose.Y().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("vision rotation position"), pose.Rotation().Degrees().to<double>());

            chassis->SetYaw(pose.Rotation().Degrees());
            chassis->ResetPose(pose);
        }
        else
        {
            auto path = PathPlannerPath::fromPathFile(param->GetPathName());
            if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
            {
                path = path.get()->flipPath();
            }
            auto pose = path.get()->getPreviewStartingHolonomicPose();

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("current x position"), pose.X().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("current y position"), pose.Y().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("current rotation position"), pose.Rotation().Degrees().to<double>());

            chassis->SetYaw(pose.Rotation().Degrees());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("GetYaw"), chassis->GetYaw().to<double>());

            chassis->ResetPose(pose);

            auto newPose = chassis->GetPose();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("new x position"), newPose.X().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("new y position"), newPose.Y().to<double>());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ResetPosition"), string("new rotation position"), newPose.Rotation().Degrees().to<double>());
        }
    }
}

void ResetPositionPathPlanner::Run()
{
}

bool ResetPositionPathPlanner::IsDone()
{
    return true;
}