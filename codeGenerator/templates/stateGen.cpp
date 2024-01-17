$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_gen.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_$$_STATE_NAME_$$_StateGen.h"
#include "mechanisms/base/BaseMech.h"

#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

// Third Party Includes

using std::string;

/// @class $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen::$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen(string stateName,
                                                                                                             int stateId,
                                                                                                             $$_MECHANISM_INSTANCE_NAME_$$_gen &mech) : $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen(stateName, stateId, mech)
{
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen::Init()
{
    $$_SET_TARGET_CONTROL_$$

    Logger::GetLogger()
        ->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen"), string("init"));

    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Init();
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen"), string("run"));
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Run();
}

void $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen::Exit()
{
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Exit();
}

bool $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen::AtTarget()
{
    return $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTarget();
}
