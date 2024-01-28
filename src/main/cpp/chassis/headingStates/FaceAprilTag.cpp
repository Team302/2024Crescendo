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
#include "chassis/headingStates/FaceAprilTag.h"

#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include "configs/RobotElementNames.h"
#include "hw/interfaces/IDragonPigeon.h"

#include "utils/FMSData.h"
#include "utils/AngleUtils.h"

/// debugging
#include "utils/logging/Logger.h"

FaceAprilTag::FaceAprilTag() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_APRIL_TAG)
// m_pipelineMode(DragonLimelight::APRIL_TAG),
//     m_vision(DragonVision::GetDragonVision())
{
}

void FaceAprilTag::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::radians_per_second_t omega = units::angular_velocity::radians_per_second_t(0.0);

    units::angle::radian_t angleError = units::angle::radian_t(0.0);

    // get targetdata from the vision system
    // visionapi - update this for new dragon vision
    /*  if (m_vision->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT) != DragonLimelight::PIPELINE_MODE::APRIL_TAG)
      {
          m_vision->setPipeline(DragonLimelight::PIPELINE_MODE::APRIL_TAG);
      }

      auto targetData = m_vision->getTargetInfo();

      if ((targetData != nullptr) && (m_vision->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT) == targetData->getTargetType()))
    {
        if (!AtTargetAngle(targetData, &angleError)) */
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Angle Error (Deg)", units::angle::degree_t(angleError).to<double>());

        // omega = units::angle::radian_t(angleError * m_visionKP_Angle) / 1_s;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Omega Before Limiting (Deg Per Sec)", units::angular_velocity::degrees_per_second_t(omega).to<double>());

        omega = limitAngularVelocityToBetweenMinAndMax(omega);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Omega After Limiting (Deg Per Sec)", units::angular_velocity::degrees_per_second_t(omega).to<double>());

        chassisMovement.chassisSpeeds.omega = omega;
    }
}
//}

/*bool FaceAprilTag::AtTargetAngle(std::shared_ptr<DragonVisionTarget> targetData, units::angle::radian_t *error)
{
    if (targetData != nullptr)
    {
        units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();
        units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame();

        if (std::abs(xError.to<double>()) > 0.01)
        {
            /// Math
            // First get the pigeon angle to later get field, this is considered
            // Next, get the angle to the tag, this is considered alpha
            // Calculate alpha by taking the arc/inverse tangent of our yError and xError (robot oriented) to the tag
            // Calculate field oriented error by taking the cosine and sine of alpha + theta
            // From there, we can get the angle to the back of the node (considered beta)
            // This is calculated by taking arc/inverse tangent of our field oriented yError, divided by our field oriented xError
            // and the offset to the back of the cube node

            auto pigeon = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetPigeon(RobotElementNames::PIGEON_USAGE::PIGEON_ROBOT_CENTER);
            units::angle::degree_t robotYaw = units::angle::degree_t(pigeon->GetYaw());

            auto angleToTag = units::angle::radian_t(std::atan2(yError.to<double>(), xError.to<double>()));

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Robot Yaw Before (Deg)", robotYaw.to<double>());

            if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
            {
                robotYaw = units::angle::degree_t((180.0 - units::math::abs(robotYaw).to<double>()) * (robotYaw.to<double>() < 0.0 ? 1.0 : -1.0));
            }

            units::length::inch_t xErrorFieldOriented = xError * units::math::cos(angleToTag + robotYaw);
            units::length::inch_t yErrorFieldOriented = xError * units::math::sin(angleToTag + robotYaw);

            auto angleToBackOfNode = units::math::atan2(yErrorFieldOriented, xErrorFieldOriented + m_cubeNodeLength);

            *error = -1.0 * (robotYaw - angleToBackOfNode); // negate to turn correctly

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ANickDebugging", "Robot Yaw After (Deg)", robotYaw.to<double>());

            if (std::abs((*error).to<double>()) < m_AngularTolerance_rad)
            {
                return true;
            }
        }
    }
    return false;
}*/

units::angular_velocity::radians_per_second_t FaceAprilTag::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
{
    return units::angular_velocity::degrees_per_second_t(0.0); // TODO:  calculate when we have target
    double sign = angularVelocity.to<double>() < 0 ? -1 : 1;
}