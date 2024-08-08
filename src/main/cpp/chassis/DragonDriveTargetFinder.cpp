
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
#include <string>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Transform3d.h"
#include "units/angle.h"

#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "DragonVision/DragonVisionStructLogger.h"
#include "utils/FMSData.h"
#include "utils/FieldConstants.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using frc::Pose2d;
using frc::Pose3d;
using std::make_tuple;
using std::optional;
using std::string;
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
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    tuple<DragonDriveTargetFinder::TARGET_INFO, Pose2d> targetInfo;

    if (chassisConfig != nullptr)
    {
        auto chassis = chassisConfig->GetSwerveChassis();
        auto currentPose{Pose3d(chassis->GetPose())};

        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto data = vision->GetVisionData(item);

            if (data && item == DragonVision::VISION_ELEMENT::NOTE)
            {
                auto trans3d = data.value().transformToTarget;
                auto targetPose = currentPose + trans3d;
                units::angle::degree_t robotRelativeAngle = data.value().rotationToTarget.Z(); // value is robot to target

                if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
                    robotRelativeAngle += units::angle::degree_t(180.0);
                else if (robotRelativeAngle >= units::angle::degree_t(90.0))
                    robotRelativeAngle -= units::angle::degree_t(180.0);

                units::angle::degree_t fieldRelativeAngle = chassis->GetPose().Rotation().Degrees() + robotRelativeAngle;
                targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::VISION_BASED, frc::Pose2d(targetPose.X(), targetPose.Y(), fieldRelativeAngle));

                return targetInfo;
            }
        }

        int aprilTag = -1;
        auto fieldConstants = FieldConstants::GetInstance();
        units::length::meter_t centerYLine = fieldConstants->GetFieldElement(fieldConstants->FIELD_ELEMENT::BLUE_CENTER_STAGE).Y();

        if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
        {
            auto itr = blueMap.find(item);
            if (itr != blueMap.end())
            {
                aprilTag = itr->second;
            }
            else if (item == DragonVision::VISION_ELEMENT::STAGE)
            {
                if (chassis->GetPose().Y() > centerYLine)
                    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, fieldConstants->GetFieldElement(fieldConstants->FIELD_ELEMENT::BLUE_LEFT_STAGE).ToPose2d());
                else
                    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, fieldConstants->GetFieldElement(fieldConstants->FIELD_ELEMENT::BLUE_RIGHT_STAGE).ToPose2d());
                return targetInfo;
            }
        }
        else
        {
            auto itr = redMap.find(item);
            if (itr != redMap.end())
            {
                aprilTag = itr->second;
            }
            else if (item == DragonVision::VISION_ELEMENT::STAGE)
            {
                if (chassis->GetPose().Y() > centerYLine)
                    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, fieldConstants->GetFieldElement(fieldConstants->FIELD_ELEMENT::RED_RIGHT_STAGE).ToPose2d());
                else
                    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, fieldConstants->GetFieldElement(fieldConstants->FIELD_ELEMENT::RED_LEFT_STAGE).ToPose2d());
                return targetInfo;
            }
        }

        if (aprilTag > 0)
        {
            auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
            if (pose)
            {
                auto targetPose = pose.value();

                units::length::meter_t xTrans = (targetPose.X() - currentPose.X());
                units::length::meter_t yTrans = (targetPose.Y() - currentPose.Y());

                units::angle::degree_t rawCorrection = units::math::atan(yTrans / xTrans);

                targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED, frc::Pose2d(xTrans, yTrans, rawCorrection));

                return targetInfo;
            }
        }
    }
    auto pose2d = Pose2d();
    targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND, pose2d);
    return targetInfo;
}
tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> DragonDriveTargetFinder::GetDistance(FINDER_OPTION option,
                                                                                                         DragonVision::VISION_ELEMENT item)
{
    tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> targetInfo;
    auto type = DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND;

    units::length::meter_t dist = units::length::meter_t(0.0);

    auto odometryDist = units::length::meter_t(0.0);
    auto hasOdometry = false;

    auto visionDist = units::length::meter_t(0.0);
    auto hasVision = false;

    auto aprilTagPose = GetAprilTagPose(item);
    if (option == FINDER_OPTION::ODOMETRY_ONLY || option == FINDER_OPTION::FUSE_IF_POSSIBLE)
    {
        auto chassis = GetChassis();
        if (chassis != nullptr)
        {
            auto chassispose = chassis->GetPose();
            hasOdometry = true;
            odometryDist = chassispose.Translation().Distance(aprilTagPose.Translation());
        }
    }

    if (option == FINDER_OPTION::VISION_ONLY || option == FINDER_OPTION::FUSE_IF_POSSIBLE)
    {
        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto visionposedata = vision->GetRobotPosition();
            if (visionposedata)
            {
                auto botpose = visionposedata.value().estimatedPose.ToPose2d();
                hasVision = true;
                visionDist = botpose.Translation().Distance(aprilTagPose.Translation());
            }
        }
    }

    if (option == FINDER_OPTION::VISION_ONLY && hasVision)
    {
        dist = visionDist;
        type = TARGET_INFO::VISION_BASED;
    }
    else if (option == FINDER_OPTION::ODOMETRY_ONLY && hasOdometry)
    {
        dist = odometryDist;
        type = TARGET_INFO::ODOMETRY_BASED;
    }
    else if (option == FINDER_OPTION::FUSE_IF_POSSIBLE && hasOdometry && hasVision && std::abs((odometryDist - visionDist).value()) < m_fuseTol.value())
    {
        dist = (odometryDist + visionDist) / 2.0;
        type = TARGET_INFO::VISION_ODOMETRY_FUSED;
    }
    else if (hasVision)
    {
        dist = visionDist;
        type = TARGET_INFO::VISION_BASED;
    }
    else if (hasOdometry)
    {
        dist = odometryDist;
        type = TARGET_INFO::ODOMETRY_BASED;
    }

    targetInfo = make_tuple(type, dist);
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

SwerveChassis *DragonDriveTargetFinder::GetChassis()
{
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    if (chassisConfig != nullptr)
    {
        return chassisConfig->GetSwerveChassis();
    }
    return nullptr;
}

int DragonDriveTargetFinder::GetAprilTag(DragonVision::VISION_ELEMENT item)
{
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        auto itr = blueMap.find(item);
        if (itr != blueMap.end())
        {
            return itr->second;
        }
    }
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kRed)
    {
        auto itr = redMap.find(item);
        if (itr != redMap.end())
        {
            return itr->second;
        }
    }
    return -1;
}

frc::Pose2d DragonDriveTargetFinder::GetAprilTagPose(DragonVision::VISION_ELEMENT item)
{
    auto aprilTag = GetAprilTag(item);
    if (aprilTag > 0)
    {
        auto pose = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
        if (pose)
        {
            return pose.value().ToPose2d();
        }
    }
    return {};
}

units::angle::degree_t DragonDriveTargetFinder::AdjustRobotRelativeAngleForIntake(units::angle::degree_t angle)
{
    auto robotRelativeAngle = angle;
    if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
    {
        robotRelativeAngle += units::angle::degree_t(180.0);
    }
    else if (robotRelativeAngle >= units::angle::degree_t(90.0))
    {
        robotRelativeAngle -= units::angle::degree_t(180.0);
    }
    return robotRelativeAngle;
}
