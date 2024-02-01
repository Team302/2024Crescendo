$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/base/BaseMechMotorState.h"
#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/base/BaseMechSolenoidState.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_Base_StateGen.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_gen.h"

class $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen : public $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen
{
public:
    $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen(std::string stateName,
                                                   int stateId,
                                                   $$_MECHANISM_INSTANCE_NAME_$$_gen *mech);
    $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen() = delete;
    ~$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

private:
    $$_TARGET_DECLARATIONS_$$
};
