
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
#include "chassis/IChassis.h"
#include "auton/ZoneParams.h"
#include "mechanisms/ClimberManager/generated/ClimberManagerGen.h"
#include "mechanisms/noteManager/generated/noteManagerGen.h"
// @ADDMECH include for your mechanism

// Third Party Includes

class PrimitiveParams
{
public:
    enum VISION_ALIGNMENT
    {
        UNKNOWN = -1,
        NOTE = DragonVision::VISION_ELEMENT::NOTE,
        SPEAKER = DragonVision::VISION_ELEMENT::SPEAKER
    };
    // @ADDMECH add parameter for your mechanism state
    PrimitiveParams(PRIMITIVE_IDENTIFIER id,
                    units::time::second_t time,
                    ChassisOptionEnums::HeadingOption headingOption,
                    float heading,
                    std::string pathName,
                    ZoneParamsVector zones, // create zones parameter of type
                    VISION_ALIGNMENT visionAlignment,
                    noteManagerGen::STATE_NAMES noteState,
                    ClimberManagerGen::STATE_NAMES climberState); // create zones parameter of type ZonesParamsVector

    PrimitiveParams() = delete;
    virtual ~PrimitiveParams() = default; // Destructor

    // Some getters
    PRIMITIVE_IDENTIFIER GetID() const { return m_id; };
    units::time::second_t GetTime() const { return m_time; };
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };
    float GetHeading() const { return m_heading; };
    std::string GetPathName() const { return m_pathName; };
    ZoneParamsVector GetZones() const { return m_zones; }; // create a GetZones() method to return the instance of zones m_zones
    VISION_ALIGNMENT GetVisionAlignment() const { return m_visionAlignment; }
    noteManagerGen::STATE_NAMES GetNoteState() const { return m_noteState; }
    ClimberManagerGen::STATE_NAMES GetClimberState() const { return m_climberState; }

    // Setters
    void SetPathName(std::string path) { m_pathName = path; }
    void SetVisionAlignment(VISION_ALIGNMENT visionAlignment) { m_visionAlignment = visionAlignment; }

private:
    // Primitive Parameters
    PRIMITIVE_IDENTIFIER m_id; // Primitive ID
    units::time::second_t m_time;
    ChassisOptionEnums::HeadingOption m_headingOption;
    float m_heading;
    std::string m_pathName;

    VISION_ALIGNMENT m_visionAlignment;
    noteManagerGen::STATE_NAMES m_noteState;
    ClimberManagerGen::STATE_NAMES m_climberState;

    ZoneParamsVector m_zones;
};

typedef std::vector<PrimitiveParams *> PrimitiveParamsVector;
