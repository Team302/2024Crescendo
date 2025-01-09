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
#include "chassis/driveStates/PolarDrive.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

PolarDrive::PolarDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> PolarDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        units::length::meter_t reefXPos = units::length::meter_t(4.5); // Reef center x-coordinate, needs updating based on alliance and use constants
        units::length::meter_t reefYPos = units::length::meter_t(4.0); // Reef center y-coordinate, needs updating based on alliance and use constants

        auto chassisSpeeds = chassisMovement.chassisSpeeds;

        frc::Pose2d currentPose = m_chassis->GetPose();
        double xDiff = currentPose.X().value() - reefXPos.value();
        double yDiff = currentPose.Y().value() - reefYPos.value();

        double radius = std::hypot(xDiff, yDiff);
        double angle = std::atan2(yDiff, xDiff);

        // Adjust radius based on Vx (forward/backward motion)
        radius += chassisSpeeds.vx.value() * m_loopRate * -1; // Negative since forward decreases radius

        // Adjust angle based on Vy (lateral motion)
        double angularVelocity = chassisSpeeds.vy.value() / radius; // Angular velocity in radians per second
        angle += angularVelocity * m_loopRate;                      // Adjust angle for this loop iteration

        // Update chassisSpeeds to reflect the tangential motion
        double tangentialVelocity = chassisSpeeds.vx.value(); // Tangential velocity from Vx input
        chassisSpeeds.vx = units::velocity::meters_per_second_t(std::cos(angle) * tangentialVelocity);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(std::sin(angle) * tangentialVelocity);

        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("PolarDrive"), string("chassis"), string("nullptr"));
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void PolarDrive::Init(ChassisMovement &chassisMovement)
{
}
