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

#include "chassis/driveStates/RobotDrive.h"
#include "utils/FMSData.h"
#include "chassis\DragonDriveTargetFinder.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerTrajectory.h"

DriveToLeftStage::DriveToLeftStage(RobotDrive *robotDrive,
                                   TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner) : TrajectoryDrivePathPlanner(robotDrive),
                                                                                             m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonVision = DragonVision::GetDragonVision();
    m_dragonDriveTargetFinder = DragonDriveTargetFinder::GetInstance();
    m_trajectoryDrivePathPlanner = trajectoryDrivePathPlanner;
}

void DriveToLeftStage::Init(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();
        auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::LEFT_STAGE);
        auto type = get<0>(aprilTagInfo);
        m_targetPose2d = get<1>(aprilTagInfo);

        if (type == DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED ||
            type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED)
        {
            m_trajectory = CreateDriveToLeftStagePath(currentPose2d, m_targetPose2d);
        }
    }
}

std::array<frc::SwerveModuleState, 4> DriveToLeftStage::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    m_oldTargetPose2d = m_targetPose2d;
    auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::LEFT_STAGE);
    m_targetPose2d = get<1>(aprilTagInfo);

    if (m_targetPose2d != m_oldTargetPose2d)
    {
        Init(chassisMovement);
    }

    chassisMovement.pathplannerTrajectory = m_trajectory;
    return m_trajectoryDrivePathPlanner->UpdateSwerveModuleStates(chassisMovement);
}

pathplanner::PathPlannerTrajectory DriveToLeftStage::CreateDriveToLeftStagePath(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    if (m_dragonVision != nullptr && m_chassis != nullptr)
    {
        pathplanner::PathPlannerTrajectory trajectory;

        if (allianceColor == frc::DriverStation::kBlue)
        {
            std::vector<frc::Pose2d> poses{
                currentPose2d,
                frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() - 1.0_m), targetPose2d.Rotation()),
                targetPose2d,
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
        else if (allianceColor == frc::DriverStation::kRed)
        {
            std::vector<frc::Pose2d> poses{
                currentPose2d,
                frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() - 1.0_m), targetPose2d.Rotation()),
                targetPose2d,
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