$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "State.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_$$_STATE_NAME_$$_StateGen.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/decoratormods/$$_MECHANISM_INSTANCE_NAME_$$_$$_STATE_NAME_$$_State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"

#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

// Third Party Includes

using namespace std;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State(std::string stateName,
                                                                                                       int stateId,
                                                                                                       $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen *generatedState) : State(stateName, stateId), m_genState(generatedState)
{
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State"), string("init"));

    m_genState->Init();
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State"), string("run"));
    m_genState->Run();
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::Exit()
{
    m_genState->Exit();
}

bool $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::AtTarget()
{
    auto attarget = m_genState->AtTarget();
    return attarget;
}

bool $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State::IsTransitionCondition(bool considerGamepadTransitions) const
{
    // auto transition = m_genState->IsTransitionCondition(considerGamepadTransitions);
    // return transition;
    return (considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::EXAMPLE_MECH_FORWARD));
}
