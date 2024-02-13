
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

#pragma once

// C++ Includes
#include <memory>

// FRC includes
#include <frc/geometry/Pose2d.h>
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

// Team 302 includes
#include "mechanisms/controllers/ControlData.h"
#include <mechanisms/controllers/ControlModes.h>
#include <chassis/ChassisMovement.h>

// Third Party Includes

namespace frc
{
    struct ChassisSpeeds;
}

///	 @interface IChassis
///  @brief	    Interface for differential drives
class IChassis
{
public:
    enum CHASSIS_TYPE
    {
        DIFFERENTIAL,
        MECANUM,
        SWERVE
    };

    /// @brief      return the chassis type
    /// @returns    CHASSIS_TYPE
    virtual CHASSIS_TYPE GetType() const = 0;

    /// @brief      Run chassis
    /// @returns    void
    virtual void Drive(ChassisMovement &moveInfo) = 0;

    virtual void SetTargetHeading(units::angle::degree_t targetYaw) = 0;

    virtual void Initialize() = 0;

    virtual frc::Pose2d GetPose() const = 0;
    virtual void ResetPose(const frc::Pose2d &pose) = 0;

    virtual void UpdateOdometry() = 0;
    virtual units::length::inch_t GetWheelDiameter() const = 0;
    virtual units::length::inch_t GetTrack() const = 0;
    virtual units::velocity::meters_per_second_t GetMaxSpeed() const = 0;
    virtual units::angular_velocity::radians_per_second_t GetMaxAngularSpeed() const = 0;
    virtual units::angle::degree_t GetYaw() const = 0;

    IChassis() = default;
    virtual ~IChassis() = default;
};
