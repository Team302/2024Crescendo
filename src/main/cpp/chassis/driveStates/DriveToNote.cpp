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

// FRC Includes
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

// Team302 Includes
#include "DragonVision/DragonVision.h"
#include "chassis/SwerveChassis.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/driveStates/DriveToNote.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVisionStructs.h"

using namespace pathplanner;

DriveToNote::DriveToNote() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
DriveToNote *DriveToNote::m_instance = nullptr;
DriveToNote *DriveToNote::getInstance()
{
    if (DriveToNote::m_instance == nullptr)
    {
        DriveToNote::m_instance = new DriveToNote();
    }
    return DriveToNote::m_instance;
}
auto visiondata = DragonVision::GetDragonVision();

static units::angle::degree_t getNoteDirection()
{
    if (visiondata != nullptr)
    {
        auto notedata = visiondata->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);
        if (notedata)
        {
            auto notetransform = notedata.value().deltaToTarget;
            frc::Rotation3d notedirection3d = notetransform.Rotation();
            frc::Rotation2d notedirection2d = notedirection3d.ToRotation2d();
            units::angle::degree_t notedirectiondegrees = notedirection2d.Degrees();
            return notedirectiondegrees;
        }
    }
}

pathplanner::PathPlannerTrajectory DriveToNote::CreateDriveToNote()
{
    DriveToNote *dtnvisiondata = DriveToNote::getInstance();
    auto notedata = visiondata->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);

    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    pathplanner::PathPlannerTrajectory trajectory;

    if (visiondata != nullptr && m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();

        if (allianceColor == frc::DriverStation::kBlue)
        {
            units::angle::degree_t currentnotedirection = dtnvisiondata->getNoteDirection();
            if (currentnotedirection)
            {
                auto notetransform = notedata.value().deltaToTarget;
                auto noteDistance = frc::Pose2d(notetransform.X(), notetransform.Y(), frc::Rotation2d(currentnotedirection));
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

                return trajectory;
            }
        }
        else if (allianceColor == frc::DriverStation::kBlue)
        {
            units::angle::degree_t currentnotedirection = dtnvisiondata->getNoteDirection();
            if (currentnotedirection)
            {
                auto notetransform = notedata.value().deltaToTarget;
                auto noteDistance = frc::Pose2d(notetransform.X(), notetransform.Y(), frc::Rotation2d(currentnotedirection));
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

                return trajectory;
            }
        }
    }
}