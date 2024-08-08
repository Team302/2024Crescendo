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

// FRC Includes
#include <frc/geometry/Rotation2d.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "DragonVision/DragonVision.h"
#include "chassis/SwerveChassis.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/driveStates/DriveToRightStage.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVisionStructs.h"
#include "chassis/DragonDriveTargetFinder.h"

using namespace pathplanner;

DriveToRightStage::DriveToRightStage() : m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
DriveToRightStage *DriveToRightStage::m_instance = nullptr;
DriveToRightStage *DriveToRightStage::GetInstance()
{
    if (DriveToRightStage::m_instance == nullptr)
    {
        DriveToRightStage::m_instance = new DriveToRightStage();
    }
    return DriveToRightStage::m_instance;
}

pathplanner::PathPlannerTrajectory DriveToRightStage::CreateDriveToRightStage()
{
    auto finder = DragonDriveTargetFinder::GetInstance();

    auto info = finder->GetPose(DragonVision::VISION_ELEMENT::RIGHT_STAGE);
    auto type = get<0>(info);
    auto targetRightStagePose = get<1>(info);

    frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

    pathplanner::PathPlannerTrajectory trajectory;
    if (type != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND && m_chassis != nullptr)
    {
        auto currentPose2d = m_chassis->GetPose();

        if (allianceColor == frc::DriverStation::kBlue)
        {
            auto rightStagePoseRotation = targetRightStagePose.Rotation();
            units::angle::degree_t rightStagePoseDegrees = rightStagePoseRotation.Degrees();
            auto rightStagePoseDistance = frc::Pose2d(targetRightStagePose.X(), targetRightStagePose.Y(), rightStagePoseDegrees);
            units::length::meter_t offsetX = targetRightStagePose.X() + units::length::meter_t(1);
            units::length::meter_t offsetY = targetRightStagePose.Y() + units::length::meter_t(1);
            auto offsetPoseDistanceFromCenterLine = frc::Pose2d(offsetX, offsetY, frc::Rotation2d(180_deg));

            std::vector<frc::Pose2d> poses{
                currentPose2d,
                offsetPoseDistanceFromCenterLine,
                rightStagePoseDistance};
            std::vector<frc::Translation2d> rightstagebezierPoints = PathPlannerPath::bezierFromPoses(poses);
            auto rightstagepath = std::make_shared<PathPlannerPath>(
                rightstagebezierPoints,
                PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                GoalEndState(0.0_mps, rightStagePoseDegrees));
            rightstagepath->preventFlipping = true;
            trajectory = rightstagepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());

            return trajectory;
        }
        if (allianceColor == frc::DriverStation::kRed)
        {
            auto rightStagePoseRotation = targetRightStagePose.Rotation();
            units::angle::degree_t rightStagePoseDegrees = rightStagePoseRotation.Degrees();

            auto rightStagePoseDistance = frc::Pose2d(targetRightStagePose.X(), targetRightStagePose.Y(), rightStagePoseDegrees);
            units::length::meter_t offsetX = targetRightStagePose.X() - units::length::meter_t(1);
            units::length::meter_t offsetY = targetRightStagePose.Y() - units::length::meter_t(1.5);
            auto offsetPoseDistanceFromCenterLine = frc::Pose2d(offsetX, offsetY, frc::Rotation2d(180_deg));
            std::vector<frc::Pose2d> poses{
                currentPose2d,
                offsetPoseDistanceFromCenterLine,
                rightStagePoseDistance};
            std::vector<frc::Translation2d> rightstagebezierPoints = PathPlannerPath::bezierFromPoses(poses);
            auto rightstagepath = std::make_shared<PathPlannerPath>(
                rightstagebezierPoints,
                PathConstraints(m_maxVel, m_maxAccel, m_maxAngularVel, m_maxAngularAccel),
                GoalEndState(0.0_mps, rightStagePoseDegrees));
            rightstagepath->preventFlipping = true;
            trajectory = rightstagepath->getTrajectory(m_chassis->GetChassisSpeeds(), currentPose2d.Rotation());
        }
    }
    return trajectory;
}