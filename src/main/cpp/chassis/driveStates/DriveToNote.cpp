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
    m_trajectory = CreateDriveToNote();
    chassisMovement.pathplannerTrajectory = m_trajectory;
    TrajectoryDrivePathPlanner::Init(chassisMovement);
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote()
{

    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto data = vision->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);
            if (data)
            {
                pathplanner::PathPlannerTrajectory trajectory;

                frc::Pose2d currentPose2d = m_chassis->GetPose();
                units::angle::degree_t robotRelativeAngle = data.value().rotationToTarget.Z();

                if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
                    robotRelativeAngle += units::angle::degree_t(180.0);
                else if (robotRelativeAngle >= units::angle::degree_t(90.0))
                    robotRelativeAngle -= units::angle::degree_t(180.0);

                units::angle::degree_t fieldRelativeAngle = chassis->GetPose().Rotation().Degrees() + robotRelativeAngle;
                units::length::meter_t xPos = units::length::meter_t();
                units::length::meter_t yPos = units::length::meter_t();

                if (fieldRelativeAngle > units::angle::degree_t(180)) // correcting for roll over
                    fieldRelativeAngle = units::angle::degree_t(360) - fieldRelativeAngle;
                else if (fieldRelativeAngle < units::angle::degree_t(-180))
                    fieldRelativeAngle = fieldRelativeAngle + units::angle::degree_t(360);

                if (units::math::abs(fieldRelativeAngle) <= units::angle::degree_t(90))
                {
                    xPos = (currentPose2d.X() + data.value().transformToTarget.X());
                    yPos = (currentPose2d.Y() + data.value().transformToTarget.Y());
                }
                else
                {
                    xPos = (currentPose2d.X() - data.value().transformToTarget.X());
                    yPos = (currentPose2d.Y() - data.value().transformToTarget.Y());
                }

                frc::Pose2d targetPose = frc::Pose2d(xPos, yPos, fieldRelativeAngle);
                DragonVisionStructLogger::logPose2d("CreateDriveToNote-currentPose", currentPose2d);
                DragonVisionStructLogger::logVisionData("CreateDriveToNote-VisionData", data);
                DragonVisionStructLogger::logPose2d("CreateDriveToNote-notedistance", targetPose);
                std::vector<frc::Pose2d> poses{currentPose2d, targetPose};
                std::vector<frc::Translation2d> notebezierPoints = PathPlannerPath::bezierFromPoses(poses);
                auto notepath = std::make_shared<PathPlannerPath>(notebezierPoints,
                                                                  PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                                                                  GoalEndState(0.0_mps, fieldRelativeAngle, true));
                notepath->preventFlipping = true;

                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CreateDriveToNote"), std::string("robotRelativeAngle"), robotRelativeAngle.to<double>());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CreateDriveToNote"), std::string("fieldRelativeAngle"), fieldRelativeAngle.to<double>());

                trajectory = notepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

                return trajectory;
            }
        }
    }
}