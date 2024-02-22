$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$Gen.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using std::string;
using namespace $$_MECHANISM_INSTANCE_NAME_$$States;

/// @class $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen::$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen(RobotConfigMgr::RobotIdentifier m_ActiveRobotId,
                                                                                               string stateName,
                                                                                               int stateId,
                                                                                               $$_MECHANISM_INSTANCE_NAME_$$Gen *mech) : $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen(stateName, stateId, mech), m_RobotId(m_ActiveRobotId)
{
}

void $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen::Init()
{
    $$_SET_TARGET_CONTROL_$$

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen"), string("init"));

    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Init();
}

void $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen"), string("run"));
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Run();
}

void $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen::Exit()
{
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Exit();
}

bool $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen::AtTarget()
{
    return $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTarget();
}

$$_SET_TARGET_CONTROL_FUNCTIONS_$$