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

// C++ Includes
#include <map>
#include <vector>

// FRC includes

// Team 302 includes
#include "State.h"
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/StateMgr.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;

State *myState;

/// @brief    initialize the state manager, parse the configuration file and create the states.
StateMgr::StateMgr() : m_checkGamePadTransitions(true),
                       m_mech(nullptr),
                       m_currentState(),
                       m_stateVector(),
                       m_currentStateID(0)
{
}
void StateMgr::Init(BaseMech *mech)
{
    m_mech = mech;
    if (!m_stateVector.empty())
    {
        m_currentState = m_stateVector[0];
        m_currentStateID = 0;
        m_currentState->Init();
        myState = m_currentState;
    }
}

void StateMgr::RunCommonTasks()
{
}

/// @brief  run the current state
/// @return void
void StateMgr::RunCurrentState()
{
    if (m_mech != nullptr)
    {
        CheckForStateTransition();

        // run the current state
        if (m_currentState != nullptr)
        {
            m_currentState->Run();
        }
    }
}

void StateMgr::CheckForStateTransition()
{
    CheckForSensorTransitions();
    if (m_checkGamePadTransitions)
    {
        CheckForGamepadTransitions();
    }
}

void StateMgr::CheckForSensorTransitions()
{
    auto transitions = m_currentState->GetPossibleStateTransitions();
    for (auto state : transitions)
    {
        auto transition = state->IsTransitionCondition(false);
        if (transition)
        {
            SetCurrentState(state->GetStateId(), true);
            break;
        }
    }
}

void StateMgr::CheckForGamepadTransitions()
{
    auto transitions = m_currentState->GetPossibleStateTransitions();
    for (auto state : transitions)
    {
        auto transition = state->IsTransitionCondition(true);
        if (transition)
        {
            SetCurrentState(state->GetStateId(), true);
            break;
        }
    }
}

/// @brief  set the current state, initialize it and run it
/// @return void
void StateMgr::SetCurrentState(int stateID, bool run)
{
    if (m_mech != nullptr && stateID > -1 && stateID < static_cast<int>(m_stateVector.size()))
    {
        auto state = m_stateVector[stateID];
        if (state != nullptr && state != m_currentState)
        {
            // if there are any exits that need to happen from the current state do them
            if (m_currentState != nullptr)
            {
                m_currentState->Exit();
            }

            // Transition to the new state
            m_currentState = state;
            m_currentStateID = stateID;
            m_currentState->Init();

            // Run current new state if requested
            if (run)
            {
                m_currentState->Run();
            }
        }
    }
}

void StateMgr::AddToStateVector(State *state)
{
    m_stateVector.emplace_back(state);
}

void StateMgr::LogInformation()
{
}
