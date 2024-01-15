//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes

// FRC includes

// Team 302 includes
#include <DragonVision/DragonVisionTarget.h>

// Third Party Includes

using namespace std;

DragonVisionTarget::DragonVisionTarget(DragonLimelight::PIPELINE_MODE targetType,
									   units::length::inch_t distanceFromTarget,
									   units::angle::degree_t horizontaAngleFromTarget,
									   units::angle::degree_t verticalAngleFromTarget,
									   units::length::inch_t xDistanceToTargetRobotFrame,
									   units::length::inch_t yDistanceToTargetRobotFrame,
									   int apriltagID,
									   units::time::millisecond_t latency)
{
	m_targetType = targetType;
	m_distanceFromTarget = distanceFromTarget;
	m_horizontalAngleToTarget = horizontaAngleFromTarget;
	m_verticalAngleToTarget = verticalAngleFromTarget;
	m_xDistanceToTargetRobotFrame = xDistanceToTargetRobotFrame;
	m_yDistanceToTargetRobotFrame = yDistanceToTargetRobotFrame;
	m_tagID = apriltagID;
	m_latency = latency;
}

units::length::inch_t DragonVisionTarget::getDistanceToTarget()
{
	return m_distanceFromTarget;
}

units::angle::degree_t DragonVisionTarget::getHorizontalAngleToTarget()
{
	return m_horizontalAngleToTarget;
}

units::length::inch_t DragonVisionTarget::getYdistanceToTargetRobotFrame()
{
	return m_yDistanceToTargetRobotFrame;
}

units::length::inch_t DragonVisionTarget::getXdistanceToTargetRobotFrame()
{
	return m_xDistanceToTargetRobotFrame;
}

units::angle::degree_t DragonVisionTarget::getVerticalAngleToTarget()
{
	return m_verticalAngleToTarget;
}

DragonLimelight::PIPELINE_MODE DragonVisionTarget::getTargetType()
{
	return m_targetType;
}

int DragonVisionTarget::getApriltagID()
{
	return m_tagID;
}

units::time::millisecond_t DragonVisionTarget::getLatency()
{
	return m_latency;
}
