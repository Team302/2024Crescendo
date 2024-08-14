
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
                    std::string choreoTrajectoryName,
                    ChassisOptionEnums::PathGainsType pahtgainsType,
                    ZoneParamsVector zones, // create zones parameter of type
                    VISION_ALIGNMENT visionAlignment,
                    bool changeNoteState,
                    noteManagerGen::STATE_NAMES noteState,
                    bool changeClimberState,
                    ClimberManagerGen::STATE_NAMES climberState,
                    ChassisOptionEnums::PathUpdateOption updateHeadingOption); // create zones parameter of type ZonesParamsVector

    PrimitiveParams() = delete;
    virtual ~PrimitiveParams() = default; // Destructor

    // Some getters
    PRIMITIVE_IDENTIFIER GetID() const { return m_id; };
    units::time::second_t GetTime() const { return m_time; };
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };
    float GetHeading() const { return m_heading; };
    std::string GetPathName() const { return m_pathName; };
    std::string GetTrajectoryName() const { return m_choreoTrajectoryName; };
    ChassisOptionEnums::PathGainsType GetPathGainsType() const { return m_pathGainsType; }
    ZoneParamsVector GetZones() const { return m_zones; }; // create a GetZones() method to return the instance of zones m_zones
    VISION_ALIGNMENT GetVisionAlignment() const { return m_visionAlignment; }
    ChassisOptionEnums::PathUpdateOption GetPathUpdateOption() const { return m_pathUpdateOption; }

    bool IsNoteStateChanging() const { return m_changeNoteState; }
    noteManagerGen::STATE_NAMES GetNoteState() const { return m_noteState; }

    bool IsClimberStateChanging() const { return m_changeClimberState; }
    ClimberManagerGen::STATE_NAMES GetClimberState() const { return m_climberState; }

    // Setters
    void SetPathName(std::string path) { m_pathName = path; }
    void SetVisionAlignment(VISION_ALIGNMENT visionAlignment) { m_visionAlignment = visionAlignment; }

private:
    // Primitive Parameters
    PRIMITIVE_IDENTIFIER m_id; // Primitive ID
    units::time::second_t m_time;
    ChassisOptionEnums::HeadingOption m_headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    float m_heading;
    std::string m_pathName;
    std::string m_choreoTrajectoryName;
    ChassisOptionEnums::PathGainsType m_pathGainsType;
    VISION_ALIGNMENT m_visionAlignment;
    bool m_changeNoteState;
    noteManagerGen::STATE_NAMES m_noteState;
    bool m_changeClimberState;
    ClimberManagerGen::STATE_NAMES m_climberState;
    ZoneParamsVector m_zones;
    ChassisOptionEnums::PathUpdateOption m_pathUpdateOption;
};

typedef std::vector<PrimitiveParams *> PrimitiveParamsVector;
