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

// C++ Includes
#include <tuple>
#include <string>

// FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "DragonVision/DragonVision.h"
#include "chassis/SwerveChassis.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/driveStates/DriveToNote.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVisionStructs.h"
#include "chassis/DragonDriveTargetFinder.h"

#include "utils/logging/Logger.h"
#include "utils/logging/LoggerData.h"
#include "utils/logging/LoggerEnums.h"

using namespace pathplanner;

DriveToNote::DriveToNote(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner)
    : TrajectoryDrivePathPlanner(robotDrive)
{
}

void DriveToNote::Init(ChassisMovement &chassisMovement)
{
    m_trajectory = CreateDriveToNote(chassisMovement.targetPose);
    chassisMovement.pathplannerTrajectory = m_trajectory;
    TrajectoryDrivePathPlanner::Init(chassisMovement);
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote(frc::Pose2d targetNotePose)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    // auto dragonDriveTargetFinder = DragonDriveTargetFinder::GetInstance();

    pathplanner::PathPlannerTrajectory trajectory;

    // auto info = dragonDriveTargetFinder->GetPose(DragonVision::VISION_ELEMENT::NOTE);
    // auto type = get<0>(info);
    // auto targetNotePose = get<1>(info);

    // if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED && chassis != nullptr)
    //{
    frc::Pose2d currentPose2d = m_chassis->GetPose();
    frc::Rotation2d chassisHeading = frc::Rotation2d(m_chassis->GetStoredHeading());

    // units::angle::degree_t robotRelativeAngle = targetNotePose.Rotation().Degrees();
    units::angle::degree_t robotRelativeAngle = targetNotePose.Rotation().Degrees();
    units::angle::degree_t fieldRelativeAngle = chassis->GetPose().Rotation().Degrees() - robotRelativeAngle;

    auto noteDistance = frc::Pose2d(targetNotePose.X(), targetNotePose.Y(), fieldRelativeAngle);
    std::vector<frc::Pose2d> poses{noteDistance, currentPose2d};
    std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
    auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                      PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                      GoalEndState(0.0_mps, chassisHeading));
    notepath->preventFlipping = true;
    trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
    //}
    return trajectory;
}