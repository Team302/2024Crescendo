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

        // Radial velocity: Changes radius
        double radialVelocity = chassisSpeeds.vx.value(); // Forward/backward motion directly affects radius
        radius += radialVelocity * m_loopRate * -1;       // Negative since forward decreases radius

        // Angular velocity: Changes angle
        double angularVelocity = chassisSpeeds.vy.value() / radius; // Clockwise/counter-clockwise motion
        angle += angularVelocity * m_loopRate;

        // Convert polar velocities back to Cartesian
        double vxNew = radialVelocity * std::cos(angle) - (radius * angularVelocity * std::sin(angle));
        double vyNew = radialVelocity * std::sin(angle) + (radius * angularVelocity * std::cos(angle));

        chassisSpeeds.vx = units::velocity::meters_per_second_t(vxNew);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(vyNew);

        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("PolarDrive"), string("chassis"), string("nullptr"));
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

std::string PolarDrive::GetDriveStateName() const
{
    return std::string("PolarDrive");
}

void PolarDrive::Init(ChassisMovement &chassisMovement)
{
}
