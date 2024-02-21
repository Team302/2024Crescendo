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
#include "chassis/DragonDriveTargetFinder.h"

#include "utils/FMSData.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"

DriveToAmp::DriveToAmp() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonDriveTargetFinder = DragonDriveTargetFinder::GetInstance();
}

pathplanner::PathPlannerTrajectory DriveToAmp::CreateDriveToAmpPath()
{
    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    if (m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();
        auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::AMP);
        auto type = get<0>(aprilTagInfo);
        auto targetPose2d = get<1>(aprilTagInfo);

        if (allianceColor == frc::DriverStation::kBlue)
        {
            if (type == DragonDriveTargetFinder::NOT_FOUND)
            {
                m_trajectory = DriveToAmpBlue(currentPose2d, targetPose2d);
            }
            else if (type != DragonDriveTargetFinder::NOT_FOUND)
            {
                m_trajectory = DriveToAmpBlue(currentPose2d, targetPose2d);
            }
            return m_trajectory;
        }
        else if (allianceColor == frc::DriverStation::kRed)
        {
            if (type == DragonDriveTargetFinder::NOT_FOUND)
            {
                m_trajectory = DriveToAmpRed(currentPose2d, targetPose2d);
            }
            else if (type != DragonDriveTargetFinder::NOT_FOUND)
            {
                m_trajectory = DriveToAmpRed(currentPose2d, targetPose2d);
            }
            return m_trajectory;
        }
    }
    return m_trajectory;
}

pathplanner::PathPlannerTrajectory DriveToAmp::DriveToAmpBlue(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{
    pathplanner::PathPlannerTrajectory trajectory;

    std::vector<frc::Pose2d> poses{
        currentPose2d,
        frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() - 1.0_m), targetPose2d.Rotation()),
        targetPose2d,
    };

    std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

    auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
        bezierPoints,
        pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
        pathplanner::GoalEndState(0.0_mps, targetPose2d.Rotation(), false));
    createPath->preventFlipping = true;
    trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

    return trajectory;
}
pathplanner::PathPlannerTrajectory DriveToAmp::DriveToAmpRed(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{

    pathplanner::PathPlannerTrajectory trajectory;
    std::vector<frc::Pose2d> poses{
        currentPose2d,
        frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() - 1.0_m), targetPose2d.Rotation()),
        targetPose2d,
    };

    std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

    auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
        bezierPoints,
        pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
        pathplanner::GoalEndState(0.0_mps, targetPose2d.Rotation(), false));
    createPath->preventFlipping = true;

    trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

    return trajectory;
}
