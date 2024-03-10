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
    : TrajectoryDrivePathPlanner(robotDrive), m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    m_dragonDriveTargetFinder = DragonDriveTargetFinder::GetInstance();
}

std::array<frc::SwerveModuleState, 4> DriveToNote::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{

    m_oldTargetPose = m_targetPose;
    auto aprilTagInfo = m_dragonDriveTargetFinder->GetPose(DragonVision::NOTE);
    m_targetPose = get<1>(aprilTagInfo);

    if (m_targetPose != m_oldTargetPose)
    {
        m_trajectory = CreateDriveToNote();
    }

    chassisMovement.pathplannerTrajectory = m_trajectory;

    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToNote", "Target Pose X", m_targetPose.X().to<double>());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToNote", "Target Pose Y", m_targetPose.Y().to<double>());
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToNote", "Target Rotation", m_targetPose.Rotation().Degrees().to<double>());

    return m_trajectoryDrivePathPlanner->UpdateSwerveModuleStates(chassisMovement);
}

units::angle::degree_t DriveToNote::GetNoteDirection()
{
    auto finder = DragonDriveTargetFinder::GetInstance();

    auto info = finder->GetPose(DragonVision::VISION_ELEMENT::NOTE);
    auto type = get<0>(info);
    auto targetNotePose = get<1>(info);

    if (type != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND)
    {
        auto noteRotation = targetNotePose.Rotation();
        units::angle::degree_t notedirectiondegrees = noteRotation.Degrees();
        return notedirectiondegrees;
    }
    return units::angle::degree_t(0.0); // TODO what should be returned without a target
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote()
{

    auto info = m_dragonDriveTargetFinder->GetPose(DragonVision::VISION_ELEMENT::NOTE);
    auto type = get<0>(info);
    auto targetNotePose = get<1>(info);

    pathplanner::PathPlannerTrajectory trajectory;

    if (type != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND && m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();

        units::angle::degree_t currentnotedirection = GetNoteDirection();
        if (currentnotedirection)
        {
            auto noteDistance = frc::Pose2d(targetNotePose.X(), targetNotePose.Y(), frc::Rotation2d(currentnotedirection));
            std::vector<frc::Pose2d> poses{
                currentPose2d,
                noteDistance};
            std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
            auto notepath = std::make_shared<PathPlannerPath>(
                notebezierPoints,
                PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                GoalEndState(0.0_mps, currentnotedirection));
            notepath->preventFlipping = true;
            trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
        }
    }
    return trajectory;
}