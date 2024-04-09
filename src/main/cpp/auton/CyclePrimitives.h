
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

#pragma once

// C++ Includes
#include <memory>
#include <vector>

// FRC includes
#include "frc/Timer.h"
#include "units/time.h"

// Team 302 includes
#include "chassis/ChassisOptionEnums.h"
#include "State.h"

// Third Party Includes

class AutonSelector;
class IPrimitive;
class PrimitiveFactory;
class PrimitiveParams;
class SwerveChassis;

class CyclePrimitives : public State
{
public:
	CyclePrimitives();
	virtual ~CyclePrimitives() = default;

	void Init() override;
	void Run() override;
	void Exit() override;
	bool AtTarget() override;

	AutonSelector *GetAutonSelector() const { return m_autonSelector; };

protected:
	void GetNextPrim();
	void RunDriveStop();

private:
	std::vector<PrimitiveParams *> m_primParams;
	int m_currentPrimSlot;
	IPrimitive *m_currentPrim;
	PrimitiveFactory *m_primFactory;
	IPrimitive *m_driveStop;
	AutonSelector *m_autonSelector;
	std::unique_ptr<frc::Timer> m_timer;
	units::time::second_t m_maxTime;
	bool m_isDone;
	SwerveChassis *m_chassis;
	static void SetMechanismStatesFromParam(PrimitiveParams *params);
	ChassisOptionEnums::PathUpdateOption m_updatedHeadingOption;
};
