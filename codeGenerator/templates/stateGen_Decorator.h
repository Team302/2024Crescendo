$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/controllers/MechanismTargetData.h"

#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_$$_STATE_NAME_$$_StateGen.h"

using namespace std;
class $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State : public State
{
public:
    $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State() = delete;
    $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State(std::string stateName,
                                                       int stateId,
                                                       $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen *generatedState);
    ~$$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$State() = default;
    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;
    bool IsTransitionCondition(bool considerGamepadTransitions) const override;

private:
    $$_MECHANISM_INSTANCE_NAME_$$$$_STATE_NAME_$$StateGen *m_genState;
};