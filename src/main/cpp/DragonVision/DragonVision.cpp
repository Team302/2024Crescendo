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

#include <DragonVision/DragonVision.h>
#include <DragonVision/LimelightFactory.h>
#include <utils/FMSData.h>

#include <string>
// Third Party Includes

using namespace std;

DragonVision *DragonVision::m_dragonVision = nullptr;
DragonVision *DragonVision::GetDragonVision()
{
	if (DragonVision::m_dragonVision == nullptr)
	{
		DragonVision::m_dragonVision = new DragonVision();
	}
	return DragonVision::m_dragonVision;
}

// state functions

DragonVision::DragonVision()
{
	m_DragonLimelightMap[LIMELIGHT_POSITION::FRONT] = LimelightFactory::GetLimelightFactory()->GetLimelight(LimelightUsages::PRIMARY);
	m_DragonLimelightMap[LIMELIGHT_POSITION::BACK] = LimelightFactory::GetLimelightFactory()->GetLimelight(LimelightUsages::SECONDARY);
}

bool DragonVision::setPipeline(DragonLimelight::PIPELINE_MODE mode, LIMELIGHT_POSITION position)
{
	DragonLimelight *dll = getLimelight(position);

	if (dll != nullptr)
	{
		return dll->SetPipeline(mode);
	}
	return false;
}

bool DragonVision::setPipeline(DragonLimelight::PIPELINE_MODE mode)
{
	return setPipeline(mode, LIMELIGHT_POSITION::FRONT);
}

DragonLimelight::PIPELINE_MODE DragonVision::getPipeline(LIMELIGHT_POSITION position)
{
	DragonLimelight *dll = getLimelight(position);

	if (dll != nullptr)
	{
		return dll->getPipeline();
	}

	return DragonLimelight::PIPELINE_MODE::UNKNOWN;
}

/// @brief Use this function to get the currently detected target information
/// @param position From which limelight to get the info
/// @return If a target has not been acquired, returns null, otherwise a pointer to an object containing all the information
std::shared_ptr<DragonVisionTarget> DragonVision::getTargetInfo(LIMELIGHT_POSITION position) const
{
	DragonLimelight *dll = getLimelight(position);

	if ((dll != nullptr) && (dll->HasTarget()))
	{
		std::shared_ptr<DragonVisionTarget>
			dvt = make_shared<DragonVisionTarget>(
				dll->getPipeline(),
				dll->EstimateTargetXdistance(),
				dll->GetTargetHorizontalOffset(),
				dll->GetTargetVerticalOffset(),
				dll->EstimateTargetXdistance_RelToRobotCoords(),
				dll->EstimateTargetYdistance_RelToRobotCoords(),
				dll->getAprilTagID(),
				dll->GetPipelineLatency());
		return dvt;
	}

	return nullptr;
}

std::shared_ptr<DragonVisionTarget> DragonVision::getTargetInfo() const
{
	return getTargetInfo(LIMELIGHT_POSITION::FRONT);
}

frc::Pose2d DragonVision::GetRobotPosition() const
{
	// frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();
	DragonLimelight *dllFront = getLimelight(LIMELIGHT_POSITION::FRONT);
	DragonLimelight *dllBack = getLimelight(LIMELIGHT_POSITION::BACK);

	// get alliance, if red, still get blue x,y,z, but use red rotation x,y,z

	if ((dllFront != nullptr) && (dllFront->HasTarget()))
	{
		return dllFront->GetBlueFieldPosition();
	}
	else if ((dllBack != nullptr) && (dllBack->HasTarget()))
	{
		return dllBack->GetBlueFieldPosition();
	}
	else
	{
		return frc::Pose2d{};
	}
}

frc::Pose2d DragonVision::GetRobotPosition(LIMELIGHT_POSITION position) const
{
	DragonLimelight *limelight = getLimelight(position);
	// frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

	if ((limelight != nullptr) && (limelight->HasTarget()))
	{
		return limelight->GetBlueFieldPosition();
	}
	else
	{
		return frc::Pose2d{};
	}
}

/// @brief Gets a pointer to the limelight at the specified position
/// @param position The physical location of the limelight
/// @return A pointer to the lilelight object
DragonLimelight *DragonVision::getLimelight(LIMELIGHT_POSITION position) const
{
	auto theLimeLightInfo = m_DragonLimelightMap.find(position);

	if (theLimeLightInfo != m_DragonLimelightMap.end())
	{
		return theLimeLightInfo->second;
	}

	return nullptr;
}