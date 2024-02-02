$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/decoratormods/$$_MECHANISM_INSTANCE_NAME_$$.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen.h"

using namespace std;

namespace $$_MECHANISM_INSTANCE_NAME_$$States
{
    class $$_STATE_NAME_$$State : public State
    {
    public:
        $$_STATE_NAME_$$State() = delete;
        $$_STATE_NAME_$$State(std::string stateName,
                              int stateId,
                              $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen *generatedState,
                              $$_MECHANISM_INSTANCE_NAME_$$ *mech);
        ~$$_STATE_NAME_$$State() = default;
        void Init() override;
        void Run() override;
        void Exit() override;
        bool AtTarget() override;
        bool IsTransitionCondition(bool considerGamepadTransitions) override;

    private:
        $$_MECHANISM_INSTANCE_NAME_$$AllStatesStateGen *m_genState;
        $$_MECHANISM_INSTANCE_NAME_$$ *m_mechanism;
    };
}
