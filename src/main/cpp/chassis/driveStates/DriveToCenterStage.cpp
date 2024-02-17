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
#include "chassis/driveStates/DriveToCenterStage.h"
#include "chassis/ChassisConfigMgr.h"

#include "utils/FMSData.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"

DriveToCenterStage::DriveToCenterStage() : m_chasis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chasis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonVision = DragonVision::GetDragonVision();
    m_allianceColor = FMSData::GetInstance()->GetAllianceColor();
}

pathplanner::PathPlannerTrajectory DriveToCenterStage::CreateDriveToAmpPath()
{
    if (m_dragonVision != nullptr && m_chasis != nullptr)
    {
        m_currentPose2d = m_chasis->GetPose();
        if (m_currentPose2d.X() >= 6.0_m && m_currentPose2d.X() <= 10.27_m)
        {
            return DriveToCenterStage::CreateDriveToCenterStageFromMidField();
        }
    }
}

pathplanner::PathPlannerTrajectory DriveToCenterStage::CreateDriveToCenterStageFromMidField()
{
    m_currentPose2d = m_chasis->GetPose();
    m_dragonVision = DragonVision::GetDragonVision();

    pathplanner::PathPlannerTrajectory trajectory;

    if (m_allianceColor == frc::DriverStation::kBlue)
    {
        auto data = m_dragonVision->GetVisionData(DragonVision::VISION_ELEMENT::CENTER_STAGE);

        if (data)
        {
            auto aprilTagTransform3d = std::optional<frc::Transform3d>(data.value().deltaToTarget);
            auto aprilTagDistance = frc::Pose2d(aprilTagTransform3d.value().X(), aprilTagTransform3d.value().Y(), frc::Rotation2d(180_deg));

            std::vector<frc::Pose2d> poses{
                m_currentPose2d,
                frc::Pose2d((aprilTagDistance.X() - 1.0_m), aprilTagDistance.Y(), aprilTagDistance.Rotation()),
                aprilTagDistance,
            };

            std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

            auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                bezierPoints,
                pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
            createPath->preventFlipping = true;
            trajectory = createPath->getTrajectory(m_chasis->GetChassisSpeeds(), m_currentPose2d.Rotation());

            return trajectory;
        }
    }
    else if (m_allianceColor == frc::DriverStation::kRed)
    {

        auto data = m_dragonVision->GetVisionData(DragonVision::VISION_ELEMENT::CENTER_STAGE);

        if (data)
        {
            auto aprilTagTransform3d = std::optional<frc::Transform3d>(data.value().deltaToTarget);
            auto aprilTagDistance = frc::Pose2d(aprilTagTransform3d.value().X(), aprilTagTransform3d.value().Y(), frc::Rotation2d(180_deg));

            std::vector<frc::Pose2d> poses{
                m_currentPose2d,
                frc::Pose2d((aprilTagDistance.X() - 1.0_m), aprilTagDistance.Y(), aprilTagDistance.Rotation()),
                aprilTagDistance,
            };

            std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

            auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                bezierPoints,
                pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
            createPath->preventFlipping = true;

            trajectory = createPath->getTrajectory(m_chasis->GetChassisSpeeds(), m_currentPose2d.Rotation());

            return trajectory;
        }
    }
    return trajectory;
}