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
#include "chassis/headingStates/FaceGamePiece.h"

/// debugging
#include "utils/logging/Logger.h"

FaceGamePiece::FaceGamePiece() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE)
// m_vision(DragonVision::GetDragonVision())
{
}

void FaceGamePiece::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angle::radian_t angleError = units::angle::radian_t(0.0);

    units::angular_velocity::radians_per_second_t omega = units::angular_velocity::radians_per_second_t(0.0);
}
/*/
bool FaceGamePiece::AtTargetAngle(VisionData visionData, units::angle::radian_t *angleError)
{

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
*/
// units::angular_velocity::radians_per_second_t FaceGamePiece::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
// { /*

//      double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

//      if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
//          angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

//      if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
//          angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

//      return angularVelocity;*/
// }