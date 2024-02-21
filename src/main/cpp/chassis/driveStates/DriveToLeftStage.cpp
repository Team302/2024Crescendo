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

// Team302 includes
#include "chassis/driveStates/DriveToLeftStage.h"
#include "chassis/ChassisConfigMgr.h"

#include "utils/FMSData.h"
#include "chassis\DragonDriveTargetFinder.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"

DriveToLeftStage::DriveToLeftStage() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonVision = DragonVision::GetDragonVision();
}

pathplanner::PathPlannerTrajectory DriveToLeftStage::CreateDriveToLeftStagePath()
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    auto m_currentPose2d = m_chassis->GetPose();
    m_dragonVision = DragonVision::GetDragonVision();
    auto targetfinder = DragonDriveTargetFinder::GetInstance();

    pathplanner::PathPlannerTrajectory trajectory;

    if (m_dragonVision != nullptr && m_chassis != nullptr)
    {
        if (allianceColor == frc::DriverStation::kBlue)
        {
            auto info = targetfinder->GetPose(DragonVision::VISION_ELEMENT::LEFT_STAGE);

            auto targetPose = get<1>(info);

            std::vector<frc::Pose2d> poses{
                m_currentPose2d,
                frc::Pose2d(targetPose.X(), (targetPose.Y() - 1.0_m), targetPose.Rotation()),
                targetPose,
            };

            std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

            auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                bezierPoints,
                pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
            createPath->preventFlipping = true;
            trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), m_currentPose2d.Rotation());

            return trajectory;
        }
        else if (allianceColor == frc::DriverStation::kRed)
        {

            auto info = targetfinder->GetPose(DragonVision::VISION_ELEMENT::LEFT_STAGE);

            auto targetPose = get<1>(info);

            std::vector<frc::Pose2d> poses{
                m_currentPose2d,
                frc::Pose2d(targetPose.X(), (targetPose.Y() - 1.0_m), targetPose.Rotation()),
                targetPose,
            };

            std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

            auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
                bezierPoints,
                pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(180_deg)), false);
            createPath->preventFlipping = true;

            trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), m_currentPose2d.Rotation());

            return trajectory;
        }
    }
    return trajectory;
}
