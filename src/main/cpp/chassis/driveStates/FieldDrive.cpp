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

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <math.h>

// Team302 Includes
#include "chassis/driveStates/FieldDrive.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

FieldDrive::FieldDrive(RobotDrive *robotDrive) : RobotDrive(), m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> FieldDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{

    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                                             chassisMovement.chassisSpeeds.vy,
                                                                                             chassisMovement.chassisSpeeds.omega,
                                                                                             chassis->GetPose().Rotation());

        chassisMovement.chassisSpeeds = fieldRelativeSpeeds;
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void FieldDrive::Init(ChassisMovement &chassisMovement)
{
}
