// clang-format off
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Sunday, January 28, 2024 10:39:19 AM

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"
#include "mechanisms/noteManager/generated/noteManager_Pass_StateGen.h"

using namespace std;
class noteManagerPassState : public State
{
public:
	noteManagerPassState() = delete;
	noteManagerPassState ( std::string stateName,
	                       int stateId,
	                       noteManagerPassStateGen *generatedState,
	                       noteManager *mech );
	~noteManagerPassState() = default;
	void Init() override;
	void Run() override;
	void Exit() override;
	bool AtTarget() override;
	bool IsTransitionCondition ( bool considerGamepadTransitions ) const override;

private:
	noteManagerPassStateGen *m_genState;
	noteManager *m_mechanism;
};