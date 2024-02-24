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
#include "FieldConstants.h"

FieldConstants *FieldConstants::m_instance = nullptr;
FieldConstants *FieldConstants::GetInstance()
{
    if (FieldConstants::m_instance == nullptr)
    {
        FieldConstants::m_instance = new FieldConstants();
    }
    return FieldConstants::m_instance;
}

FieldConstants::FieldConstants()
{
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CENTER_STAGE] = m_BlueCenterStage;
    fieldConstantsPoseMap[BLUE_AMP] = m_BlueAmp;
    fieldConstantsPoseMap[BLUE_RIGHT_STAGE] = m_BlueRightStage;
    fieldConstantsPoseMap[BLUE_LEFT_STAGE] = m_BlueLeftStage;
    fieldConstantsPoseMap[BLUE_SOURCE] = m_BlueSource;
    fieldConstantsPoseMap[BLUE_SPEAKER] = m_BlueSpeaker;
    fieldConstantsPoseMap[RED_AMP] = m_RedAmp;
    fieldConstantsPoseMap[RED_CENTER_STAGE] = m_RedCenterStage;
    fieldConstantsPoseMap[RED_LEFT_STAGE] = m_RedLeftStage;
    fieldConstantsPoseMap[RED_RIGHT_STAGE] = m_RedRightStage;
    fieldConstantsPoseMap[RED_SPEAKER] = m_RedSpeaker;
    fieldConstantsPoseMap[RED_SOURCE] = m_RedSource;
}
frc::Pose3d FieldConstants::GetFieldElement(FIELD_ELEMENT element)
{
    frc::Pose3d Pose3d = fieldConstantsPoseMap[element];
    return Pose3d;
}