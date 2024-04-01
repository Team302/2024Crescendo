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
#include "chassis/ChassisConfigMgr.h"
#include "chassis/driveStates/DriveToNote.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVisionStructs.h"
#include "DragonVision/DragonVisionStructLogger.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"
#include "chassis/SwerveChassis.h"

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
    m_chassisMovement = chassisMovement;
    m_trajectory = CreateDriveToNote();
    m_chassisMovement.pathplannerTrajectory = m_trajectory;
    TrajectoryDrivePathPlanner::Init(m_chassisMovement);
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote()
{

    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    pathplanner::PathPlannerTrajectory trajectory;

    if (chassis != nullptr)
    {
        frc::Pose2d currentPose2d = m_chassis->GetPose();
        frc::Pose2d targetPose = currentPose2d;

        auto info = DragonDriveTargetFinder::GetInstance()->GetPose(DragonVision::VISION_ELEMENT::NOTE);
        auto type = get<0>(info);
        auto data = get<1>(info);
        if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED)
        {
            targetPose = data;
            std::vector<frc::Pose2d> poses{currentPose2d, targetPose};
            std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
            auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                              PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                              GoalEndState(1.0_mps, data.Rotation().Degrees(), true));
            notepath->preventFlipping = true;

            trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
        }
        else if (frc::DriverStation::IsAutonomous())
        {
            targetPose = frc::Pose2d(currentPose2d.X(), currentPose2d.Y(), frc::Rotation2d(units::angle::degree_t(90.0)));
            std::vector<frc::Pose2d> poses{currentPose2d, targetPose};
            std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
            auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                              PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                              GoalEndState(0.0_mps, data.Rotation().Degrees(), true));
            notepath->preventFlipping = true;

            trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
        }
    }
    return trajectory;
}

bool DriveToNote::IsDone()
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    if (config != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto data = vision->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);
            if (!data.has_value())
            {
                return HasCheckedForNote();
            }
        }

        auto noteStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER);
        if (noteStateMgr != nullptr)
        {
            auto notemgr = dynamic_cast<noteManager *>(noteStateMgr);
            if (notemgr != nullptr)
            {
                if (notemgr->HasNote())
                    Logger ::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "why done", "Note Aquired");
                else if (TrajectoryDrivePathPlanner::IsDone())
                    Logger ::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "why done", "End of Trajectory");

                return notemgr->HasNote() || TrajectoryDrivePathPlanner::IsDone();
            }
        }
    }
    return true;
}

bool DriveToNote::HasCheckedForNote()
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (frc::DriverStation::IsAutonomous() && chassis != nullptr)
    {
        if (units::math::abs(units::math::abs(chassis->GetYaw()) - units::angle::degree_t(90.0)) <= units::angle::degree_t(5.0))
            return false;
        else
            return true;
    }
    else
    {
        Logger ::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "why done", "No Note Found");
        return true;
    }
}