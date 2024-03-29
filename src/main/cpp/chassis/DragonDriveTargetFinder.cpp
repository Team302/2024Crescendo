
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

tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> DragonDriveTargetFinder::GetPose(FINDER_OPTION option,
                                                                                     DragonVision::VISION_ELEMENT item)
{
    tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> targetInfo;
    auto type = static_cast<int>(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND);

    if (item == DragonVision::VISION_ELEMENT::STAGE && (option == FINDER_OPTION::ODOMETRY_ONLY || option == FINDER_OPTION::FUSE_IF_POSSIBLE))
    {
        auto targetPose = FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue ? m_blueStage : m_redStage;
        targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, targetPose);
        return targetInfo;
    }

    auto hasVisionPose = false;
    frc::Pose3d visionPose;

    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    if (chassisConfig != nullptr && (option == FINDER_OPTION::FUSE_IF_POSSIBLE || option == FINDER_OPTION::VISION_ONLY))
    {
        auto chassis = chassisConfig->GetSwerveChassis();
        if (chassis != nullptr)
        {
            auto currentPose{Pose3d(chassis->GetPose())};

            auto vision = DragonVision::GetDragonVision();
            if (vision != nullptr)
            {
                auto data = vision->GetVisionData(item);
                if (data)
                {
                    auto trans3d = data.value().transformToTarget;
                    visionPose = currentPose + trans3d;
                    hasVisionPose = true;

                    type += static_cast<int>(DragonDriveTargetFinder::TARGET_INFO::VISION_BASED);
                    if (item == DragonVision::VISION_ELEMENT::NOTE)
                    {
                        units::angle::degree_t robotRelativeAngle = data.value().rotationToTarget.Z();

                        if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
                        {
                            robotRelativeAngle += units::angle::degree_t(180.0);
                        }
                        else if (robotRelativeAngle >= units::angle::degree_t(90.0))
                        {
                            robotRelativeAngle -= units::angle::degree_t(180.0);
                        }
                        auto fieldRelativeAngle = chassis->GetPose().Rotation().Degrees() + robotRelativeAngle;
                        targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::VISION_BASED, frc::Pose2d(visionPose.X(), visionPose.Y(), fieldRelativeAngle));
                        return targetInfo;
                    }
                }
            }
        }
    }

    auto aprilTag = -1;
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        auto itr = blueMap.find(item);
        if (itr != blueMap.end())
        {
            aprilTag = itr->second;
        }
        else
        {
            auto itr = redMap.find(item);
            if (itr != redMap.end())
            {
                aprilTag = itr->second;
            }
        }
    }

    if (aprilTag > 0 && (option == FINDER_OPTION::FUSE_IF_POSSIBLE || option == FINDER_OPTION::ODOMETRY_ONLY))
    {
        auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
        if (pose)
        {
            auto odometryPose = pose.value();
            if (hasVisionPose && (option == FINDER_OPTION::FUSE_IF_POSSIBLE))
            {
                auto targetPose = visionPose.ToPose2d();
                auto dist = odometryPose.Translation().Distance(visionPose.Translation());
                if (dist < m_fuseTol)
                {
                    auto x = (visionPose.ToPose2d().X() + odometryPose.ToPose2d().X()) / 2.0;
                    auto y = (visionPose.ToPose2d().Y() + odometryPose.ToPose2d().Y()) / 2.0;
                    auto rot = (visionPose.ToPose2d().Rotation() + odometryPose.ToPose2d().Rotation()) / 2.0;
                    targetPose = Pose2d(x, y, rot);
                    type += static_cast<int>(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED);
                }
                targetInfo = make_tuple(static_cast<DragonDriveTargetFinder::TARGET_INFO>(type), targetPose);
                return targetInfo;
            }
        }
        else if (hasVisionPose) // only vision pose
        {
            targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, visionPose.ToPose2d());
            return targetInfo;
        }
    }
    else if (hasVisionPose && (option == FINDER_OPTION::FUSE_IF_POSSIBLE || option == FINDER_OPTION::VISION_ONLY)) // only vision pose
    {
        targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, visionPose.ToPose2d());
        return targetInfo;
    }

    auto pose2d = Pose2d();
    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND, pose2d);
    return targetInfo;
}

tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> DragonDriveTargetFinder::GetDistance(FINDER_OPTION option,
                                                                                                         DragonVision::VISION_ELEMENT item)
{
    tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> targetInfo;
    auto type = static_cast<int>(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND);

    units::length::meter_t dist = units::length::meter_t(0.0);

    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    if (chassisConfig != nullptr && (option == FINDER_OPTION::FUSE_IF_POSSIBLE || option == FINDER_OPTION::VISION_ONLY))
    {
        auto chassis = chassisConfig->GetSwerveChassis();
        if (chassis != nullptr)
        {
            auto currentPose{Pose3d(chassis->GetPose())};

            auto info = GetPose(option, item);
            auto type = get<0>(info);
            auto pose = get<1>(info);

            dist = pose.Translation().Distance(currentPose.ToPose2d().Translation());
        }
    }

    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND, dist);
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

            auto correction = ISwerveDriveOrientation::CalcHeadingCorrection(chassis->GetStoredHeading(), kp);
            chassisMovement.chassisSpeeds.omega += correction;
        }
    }
}
