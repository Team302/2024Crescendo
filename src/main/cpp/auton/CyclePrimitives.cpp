
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

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/Timer.h"

// Team 302 includes
#include "auton/AutonSelector.h"
#include "auton/CyclePrimitives.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveFactory.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "utils/logging/Logger.h"
#include "chassis/IChassis.h"
#include "chassis/ChassisOptionEnums.h"
#include "mechanisms/ClimberManager/generated/ClimberManagerGen.h"
#include "mechanisms/noteManager/generated/noteManagerGen.h"
#include "mechanisms/noteManager/generated/noteManagerGen.cpp"

// @ADDMECH include for your mechanism state

// Third Party Includes

using frc::DriverStation;
using frc::Timer;
using std::make_unique;
using std::string;

CyclePrimitives::CyclePrimitives() : State(string("CyclePrimitives"), 0),
									 m_primParams(),
									 m_currentPrimSlot(0),
									 m_currentPrim(nullptr),
									 m_primFactory(PrimitiveFactory::GetInstance()),
									 m_DriveStop(nullptr),
									 m_autonSelector(new AutonSelector()),
									 m_timer(make_unique<Timer>()),
									 m_maxTime(units::time::second_t(0.0)),
									 m_isDone(false)
{
}

void CyclePrimitives::Init()
{
	m_currentPrimSlot = 0; // Reset current prim
	m_primParams.clear();

	m_primParams = PrimitiveParser::ParseXML(m_autonSelector->GetSelectedAutoFile());
	if (!m_primParams.empty())
	{
		GetNextPrim();
	}
}

void CyclePrimitives::Run()
{
	if (m_currentPrim != nullptr)
	{
		m_currentPrim->Run();
		if (m_currentPrim->IsDone())
		{
			GetNextPrim();
		}
	}
	else
	{
		m_isDone = true;
		m_primParams.clear();  // clear the primitive params vector
		m_currentPrimSlot = 0; // Reset current prim slot
		RunDriveStop();
	}
}

void CyclePrimitives::Exit()
{
}

bool CyclePrimitives::AtTarget()
{
	return m_isDone;
}

void CyclePrimitives::GetNextPrim()
{
	PrimitiveParams *currentPrimParam = (m_currentPrimSlot < (int)m_primParams.size()) ? m_primParams[m_currentPrimSlot] : nullptr;

	m_currentPrim = (currentPrimParam != nullptr) ? m_primFactory->GetIPrimitive(currentPrimParam) : nullptr;
	if (m_currentPrim != nullptr)
	{
		m_currentPrim->Init(currentPrimParam);

		CyclePrimitives::SetMechanismStateFromParam(currentPrimParam);

		m_maxTime = currentPrimParam->GetTime();
		m_timer->Reset();
		m_timer->Start();
	}

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("Current Prim "), m_currentPrimSlot);

	m_currentPrimSlot++;
}

void CyclePrimitives::RunDriveStop()
{
	if (m_DriveStop == nullptr)
	{
		auto time = DriverStation::GetMatchType() != DriverStation::MatchType::kNone ? DriverStation::GetMatchTime() : units::time::second_t(15.0);
		auto params = new PrimitiveParams(DO_NOTHING, // identifier
										  time,		  // time
										  ChassisOptionEnums::HeadingOption::MAINTAIN,
										  0.0, // heading
											   //   0.0, // start drive speed
											   //   0.0, // end drive speed
										  string(),
										  // DragonCamera::PIPELINE::UNKNOWN,
										  // @ADDMECH mechanism state
										  // ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE,
										  // ExtenderStateMgr::EXTENDER_STATE::HOLD_POSITION_EXTEND,
										  // IntakeStateMgr::INTAKE_STATE::HOLD,
										  ZoneParamsVector(),
										  PrimitiveParams::VISION_ALIGNMENT::UNKNOWN,
										  // Below are dummy values
										  noteManagerGen::STATE_NAMES::STATE_OFF,
										  ClimberManagerGen::STATE_NAMES::STATE_OFF);
		m_DriveStop = m_primFactory->GetIPrimitive(params);
		m_DriveStop->Init(params);
	}
	m_DriveStop->Run();
}

void CyclePrimitives::SetMechanismStateFromParam(PrimitiveParams *params)
/*Make it so then if the parameters arent a nullptr,
grab the mechanism and set the state mgr to set the current state (We already have an enum to set it)*/
{
	if (params != nullptr)
	{
		StateMgr *sm = LoggableItem::GetCurrentState();
		if (sm != nullptr)
		{
			int state = LoggableItem()->GetCurrentState();
			StateMgr::SetCurrentState(state, true);
		}
	}
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