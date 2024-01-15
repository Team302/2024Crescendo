
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

#include <string>
#include "ctre/phoenix6/CANcoder.hpp"
#include "configs/usages/CanSensorUsage.h"
#include "units/angle.h"
#include "units/angular_velocity.h"

class DragonCanCoder
{
public:
	DragonCanCoder(std::string networkTableName,
				   CanSensorUsage::CANSENSOR_USAGE usage,
				   int canID,
				   std::string canBusName,
				   double offset,
				   bool reverse);
	virtual ~DragonCanCoder() = default;

	void SetRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue range);
	void SetMagnetOffset(double offset);
	void SetDirection(ctre::phoenix6::signals::SensorDirectionValue direction);

	units::angle::radian_t GetAbsolutePosition();
	units::angular_velocity::revolutions_per_minute_t GetVelocity();
	int GetCanId();
	std::string GetNetworkTableName() const { return m_networkTableName; }
	CanSensorUsage::CANSENSOR_USAGE GetUsage() const { return m_usage; }

private:
	std::string m_networkTableName;
	CanSensorUsage::CANSENSOR_USAGE m_usage;
	ctre::phoenix6::hardware::CANcoder m_cancoder;
};
