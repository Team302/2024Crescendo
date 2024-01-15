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
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <auton/PrimitiveParams.h>
#include "State.h"
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include <mechanisms/controllers/StateDataXmlParser.h>
#include <mechanisms/StateMgrHelper.h>
#include <mechanisms/StateStruc.h>
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;

/// @brief    initialize the state manager, parse the configuration file and create the states.
StateMgr::StateMgr() : m_checkGamePadTransitions(true),
                       m_mech(nullptr),
                       m_currentState(),
                       m_stateVector(),
                       m_currentStateID(0)
{
}
void StateMgr::Init(BaseMech *mech, const map<string, StateStruc> &stateMap)
{
    m_mech = mech;
    if (mech != nullptr)
    {
        // Parse the configuration file
        auto stateXML = make_unique<StateDataXmlParser>();
        vector<MechanismTargetData *> targetData = stateXML.get()->ParseXML(mech->GetType());

        if (targetData.empty())
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, mech->GetNetworkTableName(), mech->GetControlFileName(), string("No states"));
        }
        else
        {
            // initialize the xml string to state map
            m_stateVector.resize(stateMap.size(), nullptr);
            // create the states passing the configuration data
            auto stateId = 0;
            for (auto td : targetData)
            {
                auto stateString = td->GetStateString();
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, mech->GetNetworkTableName(), string("State") + to_string(stateId), stateString);
                auto stateStringToStrucItr = stateMap.find(stateString);
                if (stateStringToStrucItr != stateMap.end())
                {
                    auto struc = stateStringToStrucItr->second;
                    auto slot = struc.id;
                    if (m_stateVector[slot] == nullptr)
                    {
                        auto thisState = StateMgrHelper::CreateState(mech, struc, td);
                        if (thisState != nullptr)
                        {
                            m_stateVector[slot] = thisState;
                            if (struc.isDefault)
                            {
                                m_currentState = thisState;
                                m_currentStateID = slot;
                                m_currentState->Init();
                            }
                        }
                    }
                    else
                    {
                        auto msg = string("multiple mechanism state info for state");
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mech->GetNetworkTableName(), string("StateMgr::StateMgr"), msg);
                    }
                }
                else
                {
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mech->GetNetworkTableName(), string("StateMgr::StateMgr"), string("state not found"));
                }
            }
        }
    }
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

/// @brief  Get the current Parameter parm value for the state of this mechanism
/// @param PrimitiveParams* currentParams current set of primitive parameters
/// @returns int state id - -1 indicates that there is not a state to set
int StateMgr::GetCurrentStateParam(PrimitiveParams *currentParams)
{
    return -1;
}

void StateMgr::LogInformation()
{
    if (m_mech != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_mech->GetNetworkTableName(), string("current state id"), m_currentStateID);
        if (m_currentState != nullptr)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_mech->GetNetworkTableName(), string("current state"), m_currentState->GetStateName());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_mech->GetNetworkTableName(), string("current state id"), m_currentState->GetStateId());
        }
        auto index = 0;
        for (auto state : m_stateVector)
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_mech->GetNetworkTableName(), string("StateMgr: ") + to_string(index) + string(" - ") + string("state name"), state->GetStateName());
            index++;
        }
    }
}
