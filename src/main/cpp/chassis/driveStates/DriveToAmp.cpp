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
//=====================================================================================================================================================

// Team302 includes
#include "chassis/driveStates/DriveToAmp.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/headingStates/FaceTarget.h"

#include "utils/FMSData.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"

DriveToAmp::DriveToAmp() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonVision = DragonVision::GetDragonVision();
}

pathplanner::PathPlannerTrajectory DriveToAmp::CreateDriveToAmpPath()
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    pathplanner::PathPlannerTrajectory trajectory;

    if (m_dragonVision != nullptr && m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();
        m_dragonVision = DragonVision::GetDragonVision();

        if (allianceColor == frc::DriverStation::kBlue)
        {
            auto data = m_dragonVision->GetVisionData(DragonVision::VISION_ELEMENT::AMP);

            if (data)
            {
                auto aprilTagTransform3d = std::optional<frc::Transform3d>(data.value().deltaToTarget);
                auto aprilTagDistance = frc::Pose2d(aprilTagTransform3d.value().X(), aprilTagTransform3d.value().Y(), frc::Rotation2d(180_deg));

                std::vector<frc::Pose2d> poses{
                    currentPose2d,
                    frc::Pose2d(aprilTagDistance.X(), (aprilTagDistance.Y() - 1.0_m), aprilTagDistance.Rotation()),
                    aprilTagDistance,
                };

                std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

                auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                    bezierPoints,
                    pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                    pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
                createPath->preventFlipping = true;
                trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

                return trajectory;
            }
        }
        else if (allianceColor == frc::DriverStation::kRed)
        {

            auto data = m_dragonVision->GetVisionData(DragonVision::VISION_ELEMENT::AMP);

            if (data)
            {
                auto aprilTagTransform3d = std::optional<frc::Transform3d>(data.value().deltaToTarget);
                auto aprilTagDistance = frc::Pose2d(aprilTagTransform3d.value().X(), aprilTagTransform3d.value().Y(), frc::Rotation2d(180_deg));

                std::vector<frc::Pose2d> poses{
                    currentPose2d,
                    frc::Pose2d(aprilTagDistance.X(), (aprilTagDistance.Y() - 1.0_m), aprilTagDistance.Rotation()),
                    aprilTagDistance,
                };

                std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

                auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                    bezierPoints,
                    pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                    pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
                createPath->preventFlipping = true;

                trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

                return trajectory;
            }
        }
    }
    return trajectory;
}
std::optional<frc::Pose2d> DriveToAmp::GetAprilTagPose2d(frc::Pose2d chassisPose)
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();
    auto aprilTag = allianceColor == frc::DriverStation::kBlue ? FaceTarget::BLUE_AMP : FaceTarget::RED_AMP;
    if (m_dragonVision != nullptr)
    {
        auto data = m_dragonVision->GetVisionData(DragonVision::VISION_ELEMENT::AMP);
        if (data)
        {
            auto visionAprilTagTransform3d = data.value().deltaToTarget;
            auto chassisPose3d = frc::Pose3d(chassisPose);
            auto visionAprilTagPose3d = chassisPose3d.TransformBy(visionAprilTagTransform3d);
            return visionAprilTagPose3d.ToPose2d();
        }
        else
        {
            auto AprilTagPose3d = DragonVision::GetAprilTagLayout().GetTagPose(aprilTag).value();
            return AprilTagPose3d.ToPose2d();
        }
    }
    else
    {
        return std::nullopt;
    }
}