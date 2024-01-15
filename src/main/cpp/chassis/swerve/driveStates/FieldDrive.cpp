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
#include <chassis/swerve/driveStates/FieldDrive.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

FieldDrive::FieldDrive(RobotDrive *robotDrive) : RobotDrive(), m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> FieldDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VxBEFORE", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VyBEFORE", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "OmegaBEFORE", chassisMovement.chassisSpeeds.omega.to<double>());

    /*
    // Need to calcuate fudge factor for drifting when turning and driving
    auto vx = chassisMovement.chassisSpeeds.vx;
    auto vy = chassisMovement.chassisSpeeds.vy;
    auto omega = chassisMovement.chassisSpeeds.omega;
    double origianlSpeed = sqrt(pow(vx.to<double>(), 2) + pow(vy.to<double>(), 2)); // Original speed of hte chassis

    // debugging
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Orignal Chassis Direction", atan2(vx.to<double>(), vy.to<double>()) * 180 / PI);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Orignal Chassis Speed", origianlSpeed);

    if (std::abs(omega.to<double>()) > 0.1) // checks if you have any rotation
    {
        if (std::abs(vx.to<double>() > 0.1)) // if you have a x speed and rotation, add a small y speed to counter act the drift
        {
            vy -= units::velocity::meters_per_second_t(omega.to<double>() * vx.to<double>() * 0.175);
        }
        if (std::abs(vy.to<double>()) > 0.1) // if you have a y speed and rotation, add a small x speed to counter act the drift
        {
            vx -= units::velocity::meters_per_second_t(omega.to<double>() * chassisMovement.chassisSpeeds.vy.to<double>() * 0.175); // Need to use the original vx speed not new adjusted speed
        }
        double newSpeed = sqrt(pow(vx.to<double>(), 2) + pow(vy.to<double>(), 2)); // calculates new speed of the chassis
        double ratio = origianlSpeed / newSpeed;

        vx *= ratio; // normailzes the x component to the original chassis speed command
        vy *= ratio; // normailzes the y component to the original chassis speed command

        // debugging
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "UnNormalized Chassis Speed", newSpeed);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Normalized Chassis Speed", sqrt(pow(vx.to<double>(), 2) + pow(vy.to<double>(), 2)));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "New Chassis Direction", atan2(vx.to<double>(), vy.to<double>()) * 180 / PI);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VxFudge", vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VyFudge", vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "OmegaFudge", omega.to<double>());
    */

    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                                             chassisMovement.chassisSpeeds.vy,
                                                                                             chassisMovement.chassisSpeeds.omega,
                                                                                             chassis->GetPose().Rotation());

        chassisMovement.chassisSpeeds = fieldRelativeSpeeds;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "Chassis Rotation", chassis->GetPose().Rotation().Degrees().to<double>());
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VxAFTER", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "VyAFTER", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "FieldDrive", "OmegaAFTER", chassisMovement.chassisSpeeds.omega.to<double>());
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void FieldDrive::Init(ChassisMovement &chassisMovement)
{
}
