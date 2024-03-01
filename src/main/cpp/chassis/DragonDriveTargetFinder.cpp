
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

#include <optional>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Transform3d.h"

#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "utils/FMSData.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using frc::Pose2d;
using frc::Pose3d;
using std::make_tuple;
using std::optional;
using std::tuple;

DragonDriveTargetFinder *DragonDriveTargetFinder::m_instance = nullptr;
DragonDriveTargetFinder *DragonDriveTargetFinder::GetInstance()
{
    if (DragonDriveTargetFinder::m_instance == nullptr)
    {
        DragonDriveTargetFinder::m_instance = new DragonDriveTargetFinder();
    }
    return DragonDriveTargetFinder::m_instance;
}

tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> DragonDriveTargetFinder::GetPose(DragonVision::VISION_ELEMENT item)
{
    /*auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
     if (chassisConfig != nullptr)
    {

        auto chassis = chassisConfig->GetSwerveChassis();

        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto data = vision->GetVisionData(item);
            if (data)
            {
                auto currentPose{Pose3d(chassis->GetPose())};
                auto trans3d = data.value().transformToTarget;
                auto targetPose = currentPose + trans3d;

                tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> targetInfo;
                targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::VISION_BASED, targetPose.ToPose2d());

                return targetInfo;
            }
        }
    }
    */

    int aprilTag = -1;
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        auto itr = blueMap.find(item);
        if (itr != blueMap.end())
        {
            aprilTag = itr->second;
        }
    }
    else
    {
        auto itr = redMap.find(item);
        if (itr != redMap.end())
        {
            aprilTag = itr->second;
        }
    }

    if (aprilTag > 0)
    {
        auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
        if (pose)
        {
            auto pose2d = pose.value().ToPose2d();
            tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> targetInfo;
            targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, pose2d);
            return targetInfo;
        }
    }

    auto pose2d = Pose2d();
    tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> targetInfo;
    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND, pose2d);
    return targetInfo;
}

void DragonDriveTargetFinder::SetCorrection(ChassisMovement &chassisMovement,
                                            SwerveChassis *chassis,
                                            units::angle::degree_t target,
                                            double kp)
{
    chassis->SetStoredHeading(target);
    if (chassis != nullptr)
    {
        units::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;
        if (std::abs(rot.to<double>()) < 0.1)
        {
            chassisMovement.chassisSpeeds.omega = units::radians_per_second_t(0.0);
            // if (abs(chassisMovement.chassisSpeeds.vx.to<double>()) > 0.0 ||
            //     abs(chassisMovement.chassisSpeeds.vy.to<double>() > 0.0))
            // {
            auto correction = ISwerveDriveOrientation::CalcHeadingCorrection(chassis->GetStoredHeading(), kp);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "FaceSpeakerCorrection (degpersec)", correction.to<double>());
            chassisMovement.chassisSpeeds.omega += correction;
            // }
        }
    }
}