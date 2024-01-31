
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
#include <string>
#include <vector>

// FRC includes
#include "units/time.h"

// Team 302 includes
#include "auton/PrimitiveEnums.h"
#include "chassis/ChassisOptionEnums.h"
#include "DragonVision/DragonVision.h"
#include "chassis/swerve/driveStates/VisionDrive.h"
#include "chassis/IChassis.h"
#include "auton/ZoneParams.h"
// @ADDMECH include for your mechanism

// Third Party Includes

class PrimitiveParams
{
public:
    // @ADDMECH add parameter for your mechanism state
    PrimitiveParams(
        PRIMITIVE_IDENTIFIER id,
        units::time::second_t time,
        float distance,
        ChassisOptionEnums::HeadingOption headingOption,
        float heading,
        std::string pathName,
        DragonCamera::PIPELINE pipelineMode,
        ZoneParamsVector zones); // create zones parameter of type ZonesParamsVector

    PrimitiveParams() = delete;
    virtual ~PrimitiveParams() = default; // Destructor

    // Some getters
    PRIMITIVE_IDENTIFIER GetID() const { return m_id; };
    units::time::second_t GetTime() const { return m_time; };
    float GetDistance() const { return m_distance; };
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };
    float GetHeading() const { return m_heading; };
    std::string GetPathName() const { return m_pathName; };
    DragonCamera::PIPELINE GetPipelineMode() const { return m_pipelineMode; }
    ZoneParamsVector GetZones() const { return m_zones; } // create a GetZones() method to return the instance of zones m_zones

    // @ADDMECH Add methods to get the state mgr for mechanism

    // Setters
    void
    SetDistance(float distance)
    {
        m_distance = distance;
    }
    void SetPathName(std::string path) { m_pathName = path; }

private:
    // Primitive Parameters
    PRIMITIVE_IDENTIFIER m_id; // Primitive ID
    units::time::second_t m_time;
    float m_distance;
    ChassisOptionEnums::HeadingOption m_headingOption;
    float m_heading;
    float m_startDriveSpeed;
    float m_endDriveSpeed;
    std::string m_pathName;
    // @ADDMECH add attribute for your mechanism state
    DragonCamera::PIPELINE m_pipelineMode;
    ZoneParamsVector m_zones;
};

typedef std::vector<PrimitiveParams *> PrimitiveParamsVector;
