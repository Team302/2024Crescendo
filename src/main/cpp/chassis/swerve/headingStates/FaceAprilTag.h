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

// Team302 Includes
#include <chassis/swerve/headingStates/ISwerveDriveOrientation.h>
#include <DragonVision/DragonVision.h>

#include <numbers>

class FaceAprilTag : public ISwerveDriveOrientation
{
public:
    FaceAprilTag();
    ~FaceAprilTag() = default;

    void UpdateChassisSpeeds(ChassisMovement &chassisMovement) override;

private:
    units::angular_velocity::radians_per_second_t limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularSpeed);

    bool AtTargetAngle(units::angle::radian_t *error);

    DragonCamera::PIPELINE m_pipelineMode;
    DragonVision *m_vision;
    std::optional<VisionData> visionDataOptional = DragonVision::GetDragonVision()->GetVisionData();

    // Angular movement settings
    const double m_minimumOmega_radps = 0.5;
    const double m_maximumOmega_radps = 1.2;
    const double m_AngularTolerance_rad = std::numbers::pi * 2.0 / 180.0;
    const double m_inhibitXspeedAboveAngularError_rad = std::numbers::pi * 5.0 / 180.0;
    double m_visionKP_Angle = 4.0;

    const units::length::inch_t m_cubeNodeLength = units::length::inch_t(37.0);
};