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
#include "DragonVision/DragonVisionStructLogger.h"
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
    // chassisMovement.headingOption = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DriveToNote"), std::string("Init"), "True");
    m_trajectory = CreateDriveToNote(chassisMovement.targetPose);
    chassisMovement.pathplannerTrajectory = m_trajectory;
    TrajectoryDrivePathPlanner::Init(chassisMovement);
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote(frc::Pose2d targetNotePose)
{

    DragonVisionStructLogger::logPose2d("CreateDriveToNote", targetNotePose);
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    ;
    pathplanner::PathPlannerTrajectory trajectory;

    frc::Pose2d currentPose2d = m_chassis->GetPose();
    frc::Rotation2d chassisHeading = frc::Rotation2d(m_chassis->GetStoredHeading());

    units::angle::degree_t robotRelativeAngle = targetNotePose.Rotation().Degrees();

    units::length::meter_t interX = currentPose2d.X();
    units::length::meter_t interY = currentPose2d.Y();

    if (robotRelativeAngle <= units::angle::degree_t(-90.0))
    { // Intake for front and back (optimizing movement)
        robotRelativeAngle = targetNotePose.Rotation().Degrees() + units::angle::degree_t(180.0);
        interX -= units::length::meter_t(chassis->GetWheelBase());
        // interY -= units::length::meter_t(chassis->GetWheelBase());
    }
    else if (robotRelativeAngle >= units::angle::degree_t(90.0))
    {
        robotRelativeAngle = targetNotePose.Rotation().Degrees() - units::angle::degree_t(180.0);
        interX -= units::length::meter_t(chassis->GetWheelBase());
        // interY -= units::length::meter_t(chassis->GetWheelBase());    } else {
        interX += units::length::meter_t(chassis->GetWheelBase());
    }

    units::angle::degree_t fieldRelativeAngle = chassis->GetPose().Rotation().Degrees() + robotRelativeAngle;

    auto intermediateNotePose = frc::Pose2d(interX, interY, fieldRelativeAngle);
    auto finalNotePose = frc::Pose2d(targetNotePose.X(), targetNotePose.Y(), fieldRelativeAngle);
    DragonVisionStructLogger::logPose2d("CreateDriveToNote-currentPose", currentPose2d);
    DragonVisionStructLogger::logPose2d("CreateDriveToNote-intermediateNotePose", intermediateNotePose);
    DragonVisionStructLogger::logPose2d("CreateDriveToNote-notedistance", finalNotePose);
    std::vector<frc::Pose2d> poses{currentPose2d, intermediateNotePose, finalNotePose};
    std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
    auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                      PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                      GoalEndState(0.0_mps, fieldRelativeAngle, true));
    notepath->preventFlipping = true;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CreateDriveToNote"), std::string("targetNotePose.X()"), targetNotePose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CreateDriveToNote"), std::string("targetNotePose.Y()"), targetNotePose.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CreateDriveToNote"), std::string("fieldRelativeAngle"), fieldRelativeAngle.to<double>());

    trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
    //}
    return trajectory;
}