
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
#include <string>

#include <auton/PrimitiveParams.h>
#include "State.h"
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include "mechanisms/MechanismTypes.h"
#include <mechanisms/StateMgrHelper.h>
#include <mechanisms/StateStruc.h>
#include "utils/logging/Logger.h"

//@ADDMech Add includes for mech states and mech state mgr

using namespace std;

void StateMgrHelper::InitStateMgrs()
{
    //@ADDMech Add MechanismStateMgr::GetInstanceI() here
    // ArmStateMgr::GetInstance();
    // ExtenderStateMgr::GetInstance();
    // IntakeStateMgr::GetInstance();
}

void StateMgrHelper::RunCurrentMechanismStates()
{
    /**
    for (auto i = MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM + 1; i < MechanismTypes::MECHANISM_TYPE::MAX_MECHANISM_TYPES; ++i)
    {
        auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(static_cast<MechanismTypes::MECHANISM_TYPE>(i));

        string identifier("mechanism");
        identifier += to_string(i);

        auto stateMgr = mech != nullptr ? mech->GetStateMgr() : nullptr;

        identifier += " state mgr ";
        identifier += " running current state ";
        if (stateMgr != nullptr)
        {
            stateMgr->RunCurrentState();
        }
    }
    **/
}

void StateMgrHelper::SetMechanismStateFromParam(PrimitiveParams *params)
{
    /**
    if (params != nullptr)
    {
        for (auto i = MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM + 1; i < MechanismTypes::MECHANISM_TYPE::MAX_MECHANISM_TYPES; ++i)
        {
            auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(static_cast<MechanismTypes::MECHANISM_TYPE>(i));
            auto stateMgr = mech != nullptr ? mech->GetStateMgr() : nullptr;
            if (stateMgr != nullptr)
            {
                auto stateID = stateMgr->GetCurrentStateParam(params);
                if (stateID > -1)
                {
                    stateMgr->SetCurrentState(stateID, true);
                }
            }
        }
    }
    **/
}

void StateMgrHelper::SetCheckGamepadInputsForStateTransitions(bool check)
{
    /**
    for (auto i = MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM + 1; i < MechanismTypes::MECHANISM_TYPE::MAX_MECHANISM_TYPES; ++i)
    {
        auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(static_cast<MechanismTypes::MECHANISM_TYPE>(i));
        auto stateMgr = mech != nullptr ? mech->GetStateMgr() : nullptr;
        if (stateMgr != nullptr)
        {
            stateMgr->SetAreGamepadTransitionsChecked(check);
        }
    }
    **/
}

State *StateMgrHelper::CreateState(BaseMech *mech, StateStruc &stateInfo, MechanismTargetData *targetData)
{
    // auto controlData = targetData->GetController();
    // auto controlData2 = targetData->GetController2();
    // auto target = targetData->GetTarget();
    // auto secondaryTarget = targetData->GetSecondTarget();
    // auto solenoidState = targetData->GetSolenoidState();
    //  auto solenoid2State = targetData->GetSolenoid2State();
    //  auto robotPitch = targetData->GetRobotPitch();
    //  auto function1Coeff = targetData->GetFunction1Coeff();
    //  auto function2Coeff = targetData->GetFunction2Coeff();
    auto type = stateInfo.type;
    // auto xmlString = stateInfo.xmlIdentifier;
    // auto id = stateInfo.id;

    State *thisState = nullptr;
    switch (type)
    {
        // @ADDMECH Add case(s) to create your state(s)

        /**
        case StateType::ARM_STATE:
            thisState = new ArmState(xmlString, id, controlData, target);
            break;

        case StateType::MANUAL_ARM_STATE:
            thisState = new ArmManualState(xmlString, id, controlData, target);
            break;

        case StateType::EXTENDER_STATE:
            thisState = new ExtenderState(xmlString, id, controlData, target);
            break;

        case StateType::MANUAL_EXTENDER_STATE:
            thisState = new ExtenderManualState(xmlString, id, controlData, target);
            break;

        case StateType::INTAKE_STATE:
        {
            string identifier("creating");
            identifier += xmlString;
            identifier += to_string(id);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("IntakeDebugging"), identifier, "creating");
            thisState = new IntakeState(xmlString, id, controlData, controlData2, target, secondaryTarget, solenoidState);
        }
        break;
        **/

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mech->GetNetworkTableName(), string("StateMgr::StateMgr"), string("unknown state"));
        break;
    }
    return thisState;
}
