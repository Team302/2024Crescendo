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

// C++ Includes
#include <cmath>

// Team302 Includes
#include <chassis/swerve/headingStates/FaceAprilTag.h>

#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include "configs/RobotElementNames.h"
#include "hw/interfaces/IDragonPigeon.h"

#include <utils/FMSData.h>
#include <utils/AngleUtils.h>

/// debugging
#include "utils/logging/Logger.h"

FaceAprilTag::FaceAprilTag() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG),
                               m_pipelineMode(DragonCamera::PIPELINE::APRIL_TAG),
                               m_vision(DragonVision::GetDragonVision()) {}

void FaceAprilTag::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::radians_per_second_t omega = units::angular_velocity::radians_per_second_t(0.0);

    units::angle::radian_t angleError = units::angle::radian_t(0.0);
    std::optional<VisionData> optionalVisionData = m_vision->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
    // get targetdata from the vision system
    // visionapi - update this for new dragon vision
    if (m_vision->GetPipeline(DragonVision::CAMERA_POSITION::FRONT) != DragonCamera::PIPELINE::APRIL_TAG)
    {
        m_vision->SetPipeline(DragonCamera::PIPELINE::APRIL_TAG, DragonVision::CAMERA_POSITION::FRONT);
    }

    if (optionalVisionData.has_value())
    {
        if (!AtTargetAngle(visionData, &angleError))
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Angle Error (Deg)", units::angle::degree_t(angleError).to<double>());

            // omega = units::angle::radian_t(angleError * m_visionKP_Angle) / 1_s;

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Omega Before Limiting (Deg Per Sec)", units::angular_velocity::degrees_per_second_t(omega).to<double>());

            omega = limitAngularVelocityToBetweenMinAndMax(omega);

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Omega After Limiting (Deg Per Sec)", units::angular_velocity::degrees_per_second_t(omega).to<double>());

            chassisMovement.chassisSpeeds.omega = omega;
        }
    }
}

bool FaceAprilTag::AtTargetAngle(VisionData visionData, units::angle::radian_t *angleError)
{
    if (visionDataOptional.has_value())
    {

        /// Math
        // First get the pigeon angle to later get field, this is considered
        // Next, get the angle to the tag, this is considered alpha
        // Calculate alpha by taking the arc/inverse tangent of our yError and xError (robot oriented) to the tag
        // Calculate field oriented error by taking the cosine and sine of alpha + theta
        // From there, we can get the angle to the back of the node (considered beta)
        // This is calculated by taking arc/inverse tangent of our field oriented yError, divided by our field oriented xError
        // and the offset to the back of the cube node

        units::length::inch_t yError = visionData.deltaToTarget.Y();
        units::length::inch_t xError = visionData.deltaToTarget.X();

        if (std::abs(xError.to<double>()) > 0.01)
        {
            *angleError = visionData.deltaToTarget.Rotation().Z();

            if (std::abs((*angleError).to<double>()) < m_AngularTolerance_rad)
            {
                return true;
            }
        }
        return false;
    }

    return false;
}

units::angular_velocity::radians_per_second_t FaceAprilTag::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
{
    double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

    if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

    if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

    return angularVelocity;
}