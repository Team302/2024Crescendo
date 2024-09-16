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

#include <string>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

// Team302 Includes
#include "chassis/driveStates/StageDrive.h"
#include "DragonVision/DragonVision.h"
#include "chassis/DragonDriveTargetFinder.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

StageDrive::StageDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
}

std::string StageDrive::GetDriveStateName() const
{
    return std::string("StageDrive");
}

std::array<frc::SwerveModuleState, 4> StageDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    auto finder = DragonDriveTargetFinder::GetInstance();
    if (finder != nullptr)
    {
        if (m_chassis != nullptr)
        {
            std::optional<VisionData> testVisionData = DragonVision::GetDragonVision()->GetVisionData(DragonVision::VISION_ELEMENT::STAGE);
            if (testVisionData)
            {
                chassisMovement.chassisSpeeds.vy = 0_mps;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Stage Transform Y", testVisionData.value().transformToTarget.Y().to<double>());
                chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(testVisionData.value().transformToTarget.Y().to<double>() * -m_stageVisionKp);
            }
        }
    }
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void StageDrive::Init(ChassisMovement &chassisMovement)
{
}
