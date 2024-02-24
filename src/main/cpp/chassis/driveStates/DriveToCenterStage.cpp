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

#include "chassis/driveStates/DriveToCenterStage.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/ChassisMovement.h"

#include "utils/FMSData.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"

DriveToCenterStage::DriveToCenterStage(RobotDrive *robotDrive,
                                       TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner) : TrajectoryDrivePathPlanner(robotDrive),
                                                                                                 m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonDriveTargetFinder = DragonDriveTargetFinder::GetInstance();
    m_trajectoryDrivePathPlanner = trajectoryDrivePathPlanner;
}
auto TopZone = frc::Pose2d();

void DriveToCenterStage::Init(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();
        auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::CENTER_STAGE);
        auto type = get<0>(aprilTagInfo);
        m_targetPose2d = get<1>(aprilTagInfo);

        if (type == DragonDriveTargetFinder::TARGET_INFO::ODOMETRY_BASED)
        {
            m_trajectory = CreateDriveToCenterStageTopTrajectory(currentPose2d, m_targetPose2d);
        }
        else if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED)
        {
            m_trajectory = CreateDriveToCenterStageTopTrajectory(currentPose2d, m_targetPose2d);
        }
    }
}

frc::Pose2d DriveToCenterStage::GetOffset()
{
    auto currentPose2d = m_chassis->GetPose();
    DragonVision::VISION_ELEMENT currentZoneTag = DragonVision::NEAREST_APRILTAG;
    units::length::meter_t yoffset = units::length::meter_t(0);
    units::length::meter_t xoffset = units::length::meter_t(0);

    auto topZonePointOne = frc::Pose2d(units::length::meter_t(3.34), units::length::meter_t(6.87), 90_deg);
    auto topZonePointTwo = frc::Pose2d(units::length::meter_t(5.95), units::length::meter_t(6.87), 90_deg);
    auto topZonePointThree = frc::Pose2d(units::length::meter_t(3.34), units::length::meter_t(5.56), 90_deg);

    auto bottomZonePointOne = frc::Pose2d(units::length::meter_t(2.68), units::length::meter_t(3.52), 90_deg);
    auto bottomZonePointTwo = frc::Pose2d(units::length::meter_t(4.82), units::length::meter_t(3.52), 90_deg);
    auto bottomZonePointThree = frc::Pose2d(units::length::meter_t(2.68), units::length::meter_t(1.73), 90_deg);

    auto centerZonePointOne = frc::Pose2d(units::length::meter_t(6.44), units::length::meter_t(7.44), 90_deg);
    auto centerZonePointTwo = frc::Pose2d(units::length::meter_t(7.56), units::length::meter_t(7.44), 90_deg);
    auto centerZonePointThree = frc::Pose2d(units::length::meter_t(6.44), units::length::meter_t(0.8), 90_deg);
    if (topZonePointOne.X() < currentPose2d.X() & currentPose2d.X() < topZonePointTwo.X() & currentPose2d.Y() < topZonePointOne.Y() & topZonePointThree.Y() < currentPose2d.Y())
    {
        currentZoneTag = DragonVision::LEFT_STAGE;
        yoffset = units::length::meter_t(2.05);
    }
    else if (bottomZonePointOne.X() < currentPose2d.X() & currentPose2d.X() < bottomZonePointTwo.X() & currentPose2d.Y() < bottomZonePointOne.Y() & bottomZonePointThree.Y() < currentPose2d.Y())
    {
        DragonVision::VISION_ELEMENT currentZoneTag = DragonVision::RIGHT_STAGE;
        yoffset = units::length::meter_t(-2.05);
    }
    else if (centerZonePointOne.X() < currentPose2d.X() & currentPose2d.X() < centerZonePointTwo.X() & currentPose2d.Y() < centerZonePointOne.Y() & centerZonePointThree.Y() < currentPose2d.Y())
    {
        DragonVision::VISION_ELEMENT currentZoneTag = DragonVision::CENTER_STAGE;
        xoffset = units::length::meter_t(1.47);
    }
    auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(currentZoneTag);
    auto type = get<0>(aprilTagInfo);
    m_targetPose2d = get<1>(aprilTagInfo);
    auto offsetWaypoint = frc::Pose2d(m_targetPose2d.X(), m_targetPose2d.Y(), m_targetPose2d.Rotation());

    if (currentZoneTag == DragonVision::LEFT_STAGE)
    {
        auto offsetWaypoint = frc::Pose2d(m_targetPose2d.X(), m_targetPose2d.Y() + yoffset, m_targetPose2d.Rotation());
    }
    if (currentZoneTag == DragonVision::RIGHT_STAGE)
    {
        auto offsetWaypoint = frc::Pose2d(m_targetPose2d.X(), m_targetPose2d.Y() + yoffset, m_targetPose2d.Rotation());
    }
    if (currentZoneTag == DragonVision::CENTER_STAGE)
    {
        auto offsetWaypoint = frc::Pose2d(m_targetPose2d.X() + xoffset, m_targetPose2d.Y(), m_targetPose2d.Rotation());
    }

    return offsetWaypoint;
}

std::array<frc::SwerveModuleState, 4> DriveToCenterStage::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    m_oldTargetPose2d = m_targetPose2d;
    auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::CENTER_STAGE);
    m_targetPose2d = get<1>(aprilTagInfo);
    m_makeTrajectory = m_targetPose2d != m_oldTargetPose2d;

    if (m_makeTrajectory)
    {
        Init(chassisMovement);
    }

    chassisMovement.pathplannerTrajectory = m_trajectory;
    return m_trajectoryDrivePathPlanner->UpdateSwerveModuleStates(chassisMovement);
}
pathplanner::PathPlannerTrajectory DriveToCenterStage::CreateDriveToCenterStageTopTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{
    pathplanner::PathPlannerTrajectory trajectory;

    std::vector<frc::Pose2d> poses{
        currentPose2d,
        frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() + units::length::meter_t(1)), targetPose2d.Rotation()),
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
pathplanner::PathPlannerTrajectory DriveToCenterStage::CreateDriveToCenterStageBottomTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{
    pathplanner::PathPlannerTrajectory trajectory;

    std::vector<frc::Pose2d> poses{
        currentPose2d,
        frc::Pose2d(targetPose2d.X(), (targetPose2d.Y() + units::length::meter_t(1)), targetPose2d.Rotation()),
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
pathplanner::PathPlannerTrajectory DriveToCenterStage::CreateDriveToCenterStageMiddleTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d)
{
    pathplanner::PathPlannerTrajectory trajectory;

    std::vector<frc::Pose2d> poses{
        currentPose2d,
        targetPose2d};
    std::vector<frc::Translation2d> bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(poses);

    auto createPath = std::make_shared<pathplanner::PathPlannerPath>(
        bezierPoints,
        pathplanner::PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
        pathplanner::GoalEndState(0.0_mps, targetPose2d.Rotation(), false));
    createPath->preventFlipping = true;
    trajectory = createPath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

    return trajectory;
}