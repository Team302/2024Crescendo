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

// FRC Includes
#include <frc/filter/SlewRateLimiter.h>
#include "units/velocity.h"
#include "units/angle.h"

// Team302 Includes

#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/ChassisMovement.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "utils/logging/Logger.h"
#include <utils/FMSData.h>

using std::string;

RobotDrive::RobotDrive() : ISwerveDriveState::ISwerveDriveState(),
                           m_flState(),
                           m_frState(),
                           m_blState(),
                           m_brState(),
                           m_wheelbase(units::length::inch_t(20.0)),
                           m_wheeltrack(units::length::inch_t(20.0)),
                           m_maxspeed(units::velocity::feet_per_second_t(1.0))
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        m_wheelbase = chassis->GetWheelBase();
        m_wheeltrack = chassis->GetTrack();
        m_maxspeed = chassis->GetMaxSpeed();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("RobotDrive"), string("Chassis"), string("nullptr"));
    }
}

std::array<frc::SwerveModuleState, 4> RobotDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (chassisMovement.checkTipping)
    {
        DecideTipCorrection(chassisMovement);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vx", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vy", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Omega", chassisMovement.chassisSpeeds.omega.to<double>());

    // These calculations are based on Ether's Chief Delphi derivation
    // The only changes are that that derivation is based on positive angles being clockwise
    // and our codes/sensors are based on positive angles being counter clockwise.

    // A = Vx - omega * L/2
    // B = Vx + omega * L/2
    // C = Vy - omega * W/2
    // D = Vy + omega * W/2
    //
    // Where:
    // Vx is the sideways (strafe) vector
    // Vy is the forward vector
    // omega is the rotation about Z vector
    // L is the wheelbase (front to back)
    // W is the wheeltrack (side to side)
    //
    // Since our Vx is forward and Vy is strafe we need to rotate the vectors
    // We will use these variable names in the code to help tie back to the document.
    // Variable names, though, will follow C++ standards and start with a lower case letter.
    auto l = m_wheelbase;
    auto w = m_wheeltrack;

    auto vy = 1.0 * chassisMovement.chassisSpeeds.vx;
    auto vx = -1.0 * chassisMovement.chassisSpeeds.vy;
    auto omega = chassisMovement.chassisSpeeds.omega;
    /*
        units::time::second_t kLooperDt = units::time::second_t(20.0 / 1000.0);
        frc::Pose2d robot_pose_vel = frc::Pose2d(vx * kLooperDt, vy * kLooperDt, frc::Rotation2d(omega * kLooperDt));
        // frc::Twist2d twist_vel = chassis->GetPose().Log(robot_pose_vel);
        frc::Twist2d twist_vel = frc::Pose2d().Log(robot_pose_vel);
        // chassisMovement.chassisSpeeds = frc::ChassisSpeeds(twist_vel.dx / kLooperDt, twist_vel.dy / kLooperDt, twist_vel.dtheta / kLooperDt);

        // vx = twist_vel.dx / kLooperDt;
        // vy = twist_vel.dx / kLooperDt;
        // omega = twist_vel.dtheta / kLooperDt;
    */
    units::length::meter_t centerOfRotationW = (w / 2.0) - chassisMovement.centerOfRotationOffset.Y();
    units::length::meter_t centerOfRotationL = (l / 2.0) - chassisMovement.centerOfRotationOffset.X();

    units::velocity::meters_per_second_t omegaW = omega.to<double>() * centerOfRotationW / 1_s;
    units::velocity::meters_per_second_t omegaL = omega.to<double>() * centerOfRotationL / 1_s;

    auto a = vx + omegaL;
    auto b = vx - omegaL;
    auto c = vy + omegaW;
    auto d = vy - omegaW;

    // here we'll negate the angle to conform to the positive CCW convention
    m_flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    m_flState.angle = -1.0 * m_flState.angle.Degrees();
    m_flState.speed = units::velocity::meters_per_second_t(sqrt(pow(b.to<double>(), 2) + pow(d.to<double>(), 2)));
    double maxCalcSpeed = abs(m_flState.speed.to<double>());

    m_frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    m_frState.angle = -1.0 * m_frState.angle.Degrees();
    m_frState.speed = units::velocity::meters_per_second_t(sqrt(pow(b.to<double>(), 2) + pow(c.to<double>(), 2)));
    if (abs(m_frState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_frState.speed.to<double>());
    }

    m_blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    m_blState.angle = -1.0 * m_blState.angle.Degrees();
    m_blState.speed = units::velocity::meters_per_second_t(sqrt(pow(a.to<double>(), 2) + pow(d.to<double>(), 2)));
    if (abs(m_blState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_blState.speed.to<double>());
    }

    m_brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    m_brState.angle = -1.0 * m_brState.angle.Degrees();
    m_brState.speed = units::velocity::meters_per_second_t(sqrt(pow(a.to<double>(), 2) + pow(c.to<double>(), 2)));
    if (abs(m_brState.speed.to<double>()) > maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_brState.speed.to<double>());
    }

    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if (maxCalcSpeed > m_maxspeed.to<double>())
    {
        auto ratio = m_maxspeed.to<double>() / maxCalcSpeed;
        m_flState.speed *= ratio;
        m_frState.speed *= ratio;
        m_blState.speed *= ratio;
        m_brState.speed *= ratio;
    }
    return {m_flState, m_frState, m_blState, m_brState};
}

void RobotDrive::DecideTipCorrection(ChassisMovement &chassisMovement)

{
    if (frc::DriverStation::IsFMSAttached())
    {
        if (frc::DriverStation::GetMatchTime() > units::time::second_t(20.0))
        {
            CorrectForTipping(chassisMovement);
        }
    }
    else
    {
        CorrectForTipping(chassisMovement);
    }
}
void RobotDrive::CorrectForTipping(ChassisMovement &chassisMovement)
{
    // TODO: add checktipping variable to network table
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        // pitch is positive when back of robot is lifted and negative when front of robot is lifted
        // vx is positive driving forward, so if pitch is +, need to slow down
        auto pitch = chassis->GetPitch();
        if (std::abs(pitch.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * pitch.to<double>();
            chassisMovement.chassisSpeeds.vx -= adjust;
        }

        // roll is positive when the left side of the robot is lifted and negative when the right side of the robot is lifted
        // vy is positive strafing left, so if roll is +, need to strafe slower
        auto roll = chassis->GetRoll();
        if (std::abs(roll.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * roll.to<double>();
            chassisMovement.chassisSpeeds.vy -= adjust;
        }
    }
}

void RobotDrive::Init(ChassisMovement &chassisMovement)
{
}