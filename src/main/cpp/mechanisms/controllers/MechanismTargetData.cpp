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

#include <array>
#include <vector>
#include <string>

#include <mechanisms/controllers/ControlModes.h>
#include "mechanisms/controllers/MechanismTargetData.h"

using namespace std;

MechanismTargetData::MechanismTargetData(
    string state,
    string controller,
    string controller2,
    double target,
    double secondTarget,
    double robotPitch,
    double lessThanTransitiionTarget,
    std::string lessThanTransitionState,
    double equalTransitiionTarget,
    std::string equalTransitionState,
    double greaterThanTransitiionTarget,
    std::string greaterThanTransitionState,
    SOLENOID solenoid,
    SOLENOID solenoid2,
    array<double, 3> function1Coeff,
    array<double, 3> function2Coeff) : m_state(state),
                                       m_controller(controller),
                                       m_controller2(controller2),
                                       m_target(target),
                                       m_secondTarget(secondTarget),
                                       m_controlData(nullptr),
                                       m_controlData2(nullptr),
                                       m_solenoid(solenoid),
                                       m_solenoid2(solenoid2),
                                       m_robotPitch(robotPitch),
                                       m_lessThanTransitiionTarget(lessThanTransitiionTarget),
                                       m_lessThanTransitionState(lessThanTransitionState),
                                       m_equalTransitiionTarget(equalTransitiionTarget),
                                       m_equalTransitionState(equalTransitionState),
                                       m_greaterThanTransitiionTarget(greaterThanTransitiionTarget),
                                       m_greaterThanTransitionState(greaterThanTransitionState),
                                       m_function1Coeff(function1Coeff),
                                       m_function2Coeff(function2Coeff)
{
}

/// @brief update to include ControlData
/// @param [in] std::vector<ControlData*> - vector of ControlData Objects
/// @return void
void MechanismTargetData::Update(std::vector<ControlData *> data)
{
    for (auto cd : data)
    {
        // this is where the code stops working

        if (m_controller.compare(string(cd->GetIdentifier())) == 0)
        {
            m_controlData = cd;
        }
        if (m_controller2.compare(string(cd->GetIdentifier())) == 0)
        {
            m_controlData2 = cd;
        }
        if (m_controlData != nullptr && m_controlData2 != nullptr)
        {
            break;
        }
    }
    if (m_controlData2 == nullptr && m_controlData != nullptr)
    {
        m_controlData2 = m_controlData;
    }
}
