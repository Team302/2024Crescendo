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

// Team302 Includes
#include <chassis/swerve/headingStates/FaceGoalHeading.h>
#include <utils/logging/Logger.h>

FaceGoalHeading::FaceGoalHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::TOWARD_GOAL),
                                     m_vision(DragonVision::GetDragonVision())
// visionapi Review how LimelightFactory should be fixed here
{
}

void FaceGoalHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;
    if (m_vision != nullptr)
    {
        std::optional optionalData = m_vision->GetVisionData();
        if (optionalData.has_value())
        {
            VisionData validVisionData = optionalData.value();
            double rotCorrection = abs(validVisionData.deltaToTarget.RotateBy().Z().to<double>()) > 10.0 ? m_kPGoalHeadingControl : m_kPGoalHeadingControl * 2.0;
            rot += (validVisionData.deltaToTarget.) / 1_s * rotCorrection;
            units::math::atan2(validVisionData.deltaToTarget.Y, validVisionData.deltaToTarget.X);
        }
        else
        {
            // Hold position
        }
    }
}