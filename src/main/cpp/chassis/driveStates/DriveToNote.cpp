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
//=====================================================================================================================================================

// FRC Includes
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Rotation2d.h>

// Team302 Includes
#include "DragonVision/DragonVision.h"
#include "chassis/driveStates/DriveToNote.h"
#include "DragonVision/DragonVisionStructs.h"

DriveToNote *DriveToNote::m_instance = nullptr;
DriveToNote *DriveToNote::getInstance()
{
    if (DriveToNote::m_instance == nullptr)
    {
        DriveToNote::m_instance = new DriveToNote();
    }
    return DriveToNote::m_instance;
}
auto visiondata = DragonVision::GetDragonVision();

static units::angle::degree_t getNoteDirection()
{
    if (visiondata != nullptr)
    {
        auto notedata = visiondata->GetVisionData(DragonVision::VISION_ELEMENT::NOTE);
        if (notedata)
        {
            frc::Transform3d notetransform = notedata.value().deltaToTarget;
            frc::Rotation3d notedirection3d = notetransform.Rotation();
            frc::Rotation2d notedirection2d = notedirection3d.ToRotation2d();
            units::angle::degree_t notedirectiondegrees = notedirection2d.Degrees();
            return notedirectiondegrees;
        }
    }
}

void DriveToNote::Init()
{
    DriveToNote *dtnvisiondata = DriveToNote::getInstance();
    units::angle::degree_t currentnotedirection = dtnvisiondata->getNoteDirection();
}