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

#include <optional>

// Team302 Includes
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/headingStates/FaceGamePiece.h"

/// debugging
#include "utils/logging/Logger.h"

FaceGamePiece::FaceGamePiece() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE)
{
}

void FaceGamePiece::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto vision = DragonVision::GetDragonVision();
        if (vision != nullptr)
        {
            auto data = vision->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);
            if (data)
            {
                auto target = data.value().deltaToTarget;
                auto rotation = target.Rotation();
                auto angle = rotation.ToRotation2d().Degrees();
                chassis->SetStoredHeading(angle);
            }
        }
    }
    return false;
}

/** units::angular_velocity::radians_per_second_t FaceGamePiece::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
 { 

      double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

      if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
          angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

      if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
          angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

      return angularVelocity;
 
}**/
