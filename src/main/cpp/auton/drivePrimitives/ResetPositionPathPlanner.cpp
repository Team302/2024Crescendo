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
#include <unistd.h>

// Team 302 includes
#include "auton/drivePrimitives/AutonUtils.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "auton/drivePrimitives/ResetPositionPathPlanner.h"
#include "auton/PrimitiveParams.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
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
        auto path = AutonUtils::GetPathFromPathFile(param->GetPathName());
        if (AutonUtils::IsValidPath(path))
        {
            auto initialPose = path.get()->getPreviewStartingHolonomicPose();

            auto vision = DragonVision::GetDragonVision();

            auto visionPosition = vision->GetRobotPosition();
            auto hasVisionPose = visionPosition.has_value();
            auto initialRot = hasVisionPose ? visionPosition.value().estimatedPose.ToPose2d().Rotation().Degrees() : initialPose.Rotation().Degrees();

            // use the path angle as an initial guess for the MegaTag2 calc; chassis is most-likely 0.0 right now which may cause issues based on color
            auto megaTag2Position = vision->GetRobotPositionMegaTag2(initialRot, // chassis->GetYaw(), // mtAngle.Degrees(),
                                                                     units::angular_velocity::degrees_per_second_t(0.0),
                                                                     units::angle::degree_t(0.0),
                                                                     units::angular_velocity::degrees_per_second_t(0.0),
                                                                     units::angle::degree_t(0.0),
                                                                     units::angular_velocity::degrees_per_second_t(0.0));

            auto hasMegaTag2Position = megaTag2Position.has_value() && std::abs(chassis->GetRotationRateDegreesPerSecond()) < m_maxAngularVelocityDegreesPerSecond;

            // Check to see if current post is within 1 meter (distanceThreshold) of path position (initialPose), if it is, don't reset pose
            auto poseDiff = chassis->GetPose().Translation().Distance(initialPose.Translation());
            bool poseNeedsUpdating = poseDiff > distanceThreshold;

            if (poseNeedsUpdating)
            {
                if (hasMegaTag2Position)
                {
                    ResetPose(megaTag2Position.value().estimatedPose.ToPose2d());
                }
                else if (hasVisionPose)
                {
                    ResetPose(visionPosition.value().estimatedPose.ToPose2d());
                }
                else
                {
                    ResetPose(initialPose);
                }
            }
        }
    }
}

void ResetPositionPathPlanner::ResetPose(Pose2d pose)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        chassis->SetYaw(pose.Rotation().Degrees());
        chassis->ResetPose(pose);
    }
}

void ResetPositionPathPlanner::Run()
{
}

bool ResetPositionPathPlanner::IsDone()
{
    return true;
}