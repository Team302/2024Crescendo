$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include <string>

#include "State.h"

#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/base/BaseMechSolenoidState.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$Gen.h"
#include "configs/RobotConfigMgr.h"

namespace $$_MECHANISM_INSTANCE_NAME_$$States
{
    class $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen : public $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen
    {
    public:
        $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen(RobotConfigMgr::RobotIdentifier m_ActiveRobotId,
                                                       std::string stateName,
                                                       int stateId,
                                                       $$_MECHANISM_INSTANCE_NAME_$$Gen *mech);
        $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen() = delete;
        ~$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen() = default;

        void Init() override;
        void Run() override;
        void Exit() override;
        bool AtTarget() override;

    private:
        RobotConfigMgr::RobotIdentifier m_RobotId;

        $$_TARGET_DECLARATIONS_$$

        $$_SET_TARGET_CONTROL_FUNCTION_DECLS_$$
    };
}
