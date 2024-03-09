
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
#include <vector>

// Team 302 includes
#include "State.h"
#include "utils/logging/LoggableItem.h"

// forward declare
class BaseMech;
class PrimitiveParams;

// Third Party Includes

class StateMgr : public LoggableItem
{
public:
    StateMgr();
    ~StateMgr() = default;
    void Init(BaseMech *mech);

    virtual void RunCommonTasks();

    /// @brief  run the current state
    /// @return void
    virtual void RunCurrentState();

    /// @brief  return the current state
    /// @return int - the current state
    inline int GetCurrentState() const { return m_currentStateID; };
    inline State *GetSpecifiedState(unsigned int stateID) const { return stateID < m_stateVector.size() ? m_stateVector[stateID] : nullptr; };
    inline State *GetCurrentStatePtr() const { return m_stateVector[m_currentStateID]; };

    void LogInformation() override;

    void SetAreGamepadTransitionsChecked(bool checkGamepadTransitions) { m_checkGamePadTransitions = checkGamepadTransitions; }

    void AddToStateVector(State *state);

    /// @brief  set the current state, initialize it and run it
    /// @param [in]     int - state to set
    /// @param [in]     run - true means run, false just initialize it
    /// @return void
    virtual void SetCurrentState(int state, bool run);

protected:
    virtual void CheckForStateTransition();
    virtual void CheckForSensorTransitions();
    virtual void CheckForGamepadTransitions();

    bool m_checkGamePadTransitions;

    std::vector<State *> GetStateVector() { return m_stateVector; };

private:
    BaseMech *m_mech;
    State *m_currentState;
    std::vector<State *> m_stateVector;
    int m_currentStateID;
};
