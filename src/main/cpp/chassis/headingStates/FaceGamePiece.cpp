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
#include "utils/AngleUtils.h"
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
                auto rotation = data.value().rotationToTarget;

                auto robotRelativeAngle = units::angle::degree_t(rotation.Z());
                units::angle::degree_t fieldRelativeAngle = chassis->GetPose().Rotation().Degrees();

                if (robotRelativeAngle <= units::angle::degree_t(-90.0)) // Intake for front and back (optimizing movement)
                {
                    robotRelativeAngle += units::angle::degree_t(180.0);
                    fieldRelativeAngle -= robotRelativeAngle;
                }
                else if (robotRelativeAngle >= units::angle::degree_t(90.0))
                {
                    robotRelativeAngle -= units::angle::degree_t(180.0);
                    fieldRelativeAngle -= robotRelativeAngle;
                }

                fieldRelativeAngle += robotRelativeAngle;

                chassis->SetStoredHeading(fieldRelativeAngle);

                chassisMovement.chassisSpeeds.omega += CalcHeadingCorrection(fieldRelativeAngle, m_kP);

                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DriveToNote"), std::string("Relative Rot"), robotRelativeAngle.to<double>());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DriveToNote"), std::string("Filed Rot"), fieldRelativeAngle.to<double>());
            }
        }
    }
}
