$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/decoratormods/$$_STATE_NAME_$$State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace $$_MECHANISM_INSTANCE_NAME_$$States;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_STATE_NAME_$$State::$$_STATE_NAME_$$State(std::string stateName,
                                             int stateId,
                                             $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen *generatedState,
                                             $$_MECHANISM_INSTANCE_NAME_$$ *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void $$_STATE_NAME_$$State::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("init"));

    m_genState->Init();
}

void $$_STATE_NAME_$$State::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("run"));
    m_genState->Run();
}

void $$_STATE_NAME_$$State::Exit()
{
    m_genState->Exit();
}

bool $$_STATE_NAME_$$State::AtTarget()
{
    auto attarget = m_genState->AtTarget();
    return attarget;
}

bool $$_STATE_NAME_$$State::IsTransitionCondition(bool considerGamepadTransitions)
{
    // To get the current state use m_mechanism->GetCurrentState()

    return (considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::EXAMPLE_MECH_FORWARD));
}
