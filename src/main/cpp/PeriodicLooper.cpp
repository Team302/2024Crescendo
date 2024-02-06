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
//======================================================\==============================================================================================

#include <PeriodicLooper.h>

#include <vector>
#include "mechanisms/base/StateMgr.h"

using std::vector;

PeriodicLooper *PeriodicLooper::m_instance = nullptr;

PeriodicLooper *PeriodicLooper::GetInstance()
{
    if (PeriodicLooper::m_instance == nullptr)
    {
        PeriodicLooper::m_instance = new PeriodicLooper();
    }
    return PeriodicLooper::m_instance;
}

PeriodicLooper::PeriodicLooper() : m_auton(),
                                   m_teleop(),
                                   m_simulation(),
                                   m_setGamepadForAuton(false),
                                   m_setGamepadForTeleop(false),
                                   m_setGamepadForSimulation(false)
{
}

PeriodicLooper::~PeriodicLooper()
{
    m_auton.clear();
    m_teleop.clear();
    m_simulation.clear();
}

void PeriodicLooper::RegisterAuton(StateMgr *mgr)
{
    m_auton.emplace_back(mgr);
}

void PeriodicLooper::RegisterTeleop(StateMgr *mgr)
{
    m_teleop.emplace_back(mgr);
}

void PeriodicLooper::RegisterSimulation(StateMgr *mgr)
{
    m_simulation.emplace_back(mgr);
}

void PeriodicLooper::RegisterAll(StateMgr *mgr)
{
    m_auton.emplace_back(mgr);
    m_teleop.emplace_back(mgr);
    m_simulation.emplace_back(mgr);
}

void PeriodicLooper::AutonRunCurrentState()
{
    if (!m_setGamepadForAuton)
    {
        SetGamePadTransitions(m_auton, false);
        m_setGamepadForAuton = true;
    }
    RunCurrentStates(m_auton);
}

void PeriodicLooper::TeleopRunCurrentState()
{
    if (!m_setGamepadForTeleop)
    {
        SetGamePadTransitions(m_teleop, true);
        m_setGamepadForTeleop = true;
    }
    RunCurrentStates(m_teleop);
}

void PeriodicLooper::SimulationRunCurrentState()
{
    if (!m_setGamepadForSimulation)
    {
        SetGamePadTransitions(m_simulation, true);
        m_setGamepadForSimulation = true;
    }
    RunCurrentStates(m_simulation);
}

void PeriodicLooper::SetGamePadTransitions(vector<StateMgr *> mgrs, bool checkSw)
{
    for (auto mgr : mgrs)
    {
        mgr->SetAreGamepadTransitionsChecked(checkSw);
    }
}

void PeriodicLooper::RunCurrentStates(vector<StateMgr *> mgrs)
{
    for (auto mgr : mgrs)
    {
        mgr->RunCommonTasks();
        mgr->RunCurrentState();
    }
}
