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

#include "units/angle.h"

// Team302 Includes
#include "chassis/ChassisOptionEnums.h"
#include "chassis/ChassisMovement.h"

class ISwerveDriveOrientation
{
public:
    ISwerveDriveOrientation() = delete;
    ~ISwerveDriveOrientation() = default;

    ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption headingOption);

    /// @brief Updated incoming chassis speeds to do heading action, precursor to ISwerveDriveState
    /// @param [in] ChassisMovement& chassisMovement - Incomign chassis speeds to edit for heading option
    void virtual UpdateChassisSpeeds(ChassisMovement &chassisMovement) = 0;

    /// @brief Calculate heading correction
    /// @param [in] rot - incoming rotation to correct for
    /// @param [in] kP - porportional constant to correct with
    static units::angular_velocity::degrees_per_second_t CalcHeadingCorrection(units::angle::degree_t targetAngle, double kP);

    /// @brief Returns the heading option
    /// @return ChassisOptionEnums::HeadingOption - current heading option
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };

protected:
    ChassisOptionEnums::HeadingOption m_headingOption;
    units::angle::degree_t m_storedYaw;

    const int m_fineSlot = 0;
    const int m_coarseSlot = 1;
    const units::angle::degree_t m_fineCoarseAngle = units::angle::degree_t(5.0);

    const double kPMaintain[2] = {7.0, 5.0};
    const double kPSpecifiedHeading[2] = {10.0, 6.0};
};