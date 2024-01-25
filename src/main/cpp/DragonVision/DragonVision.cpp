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

// Team 302 includes

#include "DragonVision/DragonVision.h"
#include "utils/FMSData.h"

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

frc::AprilTagFieldLayout DragonVision::GetAprilTagLayout()
{
	if (m_aprilTagLayout != frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo))
	{
		m_aprilTagLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
	}
	return m_aprilTagLayout;
}

DragonVision::DragonVision()
{
}

void DragonVision::AddCamera(DragonCamera *camera, CAMERA_POSITION position)
{
	m_DragonCameraMap[position] = camera;
}

std::optional<VisionData> DragonVision::GetVisionData(VISION_ELEMENT element)
{
	// logic for selecting which camera
	DragonCamera *selectedcam = nullptr;
	if (m_DragonCameraMap[FRONT]->GetAprilTagID() != -1)
		selectedcam = m_DragonCameraMap[FRONT];

	else if (m_DragonCameraMap[BACK]->GetAprilTagID() != -1)
		selectedcam = m_DragonCameraMap[BACK];

	if (element == VISION_ELEMENT::NOTE)
	{
		/*units::angle::degree_t yaw = m_DragonCameraMap[BACK_INTAKE]->GetTargetYawRobotFrame();
		units::angle::degree_t pitch = m_DragonCameraMap[BACK_INTAKE]->GetTargetPitchRobotFrame();

		frc::Rotation3d rotation3d = {units::angle::degree_t(0.0), pitch, yaw};

		unit::length::meter_t xDistance = m_DragonCameraMap[BACK_INTAKE]->EstimateTargetXDistance_RelToRobotCoords();
		unit::length::meter_t yDistance = m_DragonCameraMap[BACK_INTAKE]->EstimateTargetYDistance_RelToRobotCoords();
		unit::length::meter_t zDistance = m_DragonCameraMap[BACK_INTAKE]->EstimateTargetZDistance_RelToRobotCoords();

		//need to verify that the translation given these values correctly gets the right yaw,pitch,roll
		//may need to use translation constructor with rotation 3d instead of x,y,z
		frc::Translation3d translation3d = {xDistance, yDistance, zDistance};
		VisionData translation = {translation3d};
		return translation;*/
	}
	else
	{
		// only for stage
		// get camera data from both cameras
		/* switch(tagId){ red case 11, 12, 13
						 blue case 14, 15, 16}
		*/

		// frc::Pose3d robotPose = m_dragonCamera->GetFieldPosition();
		// frc::Pose3d targetElementPose = fieldConstants.Get(ELEMENT);
		// VisionData transform = robotPose - targetElementPose;
		// VisionData translation = transform.translation;
		// return translation;
	}

	// if we don't see any vision targets, return null optional
	return std::nullopt;
}

std::optional<VisionPose> DragonVision::GetRobotPosition()
{
	// if we aren't able to calculate our pose from vision, return a null optional
	return std::nullopt;
}

// bool DragonVision::setPipeline(DragonLimelight::PIPELINE_MODE mode, CAMERA_POSITION position)
// {
// 	DragonLimelight *dll = getLimelight(position);

// 	if (dll != nullptr)
// 	{
// 		return dll->SetPipeline(mode);
// 	}
// 	return false;
// }

// bool DragonVision::setPipeline(DragonLimelight::PIPELINE_MODE mode)
// {
// 	return setPipeline(mode, CAMERA_POSITION::FRONT);
// }

// DragonLimelight::PIPELINE_MODE DragonVision::getPipeline(CAMERA_POSITION position)
// {
// 	DragonLimelight *dll = getLimelight(position);

// 	if (dll != nullptr)
// 	{
// 		return dll->getPipeline();
// 	}

// 	return DragonLimelight::PIPELINE_MODE::UNKNOWN;
// }

/// @brief Use this function to get the currently detected target information
/// @param position From which limelight to get the info
/// @return If a target has not been acquired, returns null, otherwise a pointer to an object containing all the information
// std::shared_ptr<DragonVisionTarget> DragonVision::getTargetInfo(CAMERA_POSITION position) const
// {
// 	DragonLimelight *dll = getLimelight(position);

// 	if ((dll != nullptr) && (dll->HasTarget()))
// 	{
// 		std::shared_ptr<DragonVisionTarget>
// 			dvt = make_shared<DragonVisionTarget>(
// 				dll->getPipeline(),
// 				dll->EstimateTargetXdistance(),
// 				dll->GetTargetHorizontalOffset(),
// 				dll->GetTargetVerticalOffset(),
// 				dll->EstimateTargetXdistance_RelToRobotCoords(),
// 				dll->EstimateTargetYdistance_RelToRobotCoords(),
// 				dll->getAprilTagID(),
// 				dll->GetPipelineLatency());
// 		return dvt;
// 	}

// 	return nullptr;
// }

// std::shared_ptr<DragonVisionTarget> DragonVision::getTargetInfo() const
// {
// 	return getTargetInfo(CAMERA_POSITION::FRONT);
// }

// frc::Pose2d DragonVision::GetRobotPosition() const
// {
// 	// frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();
// 	DragonLimelight *dllFront = getLimelight(CAMERA_POSITION::FRONT);
// 	DragonLimelight *dllBack = getLimelight(CAMERA_POSITION::BACK);

// 	// get alliance, if red, still get blue x,y,z, but use red rotation x,y,z

// 	if ((dllFront != nullptr) && (dllFront->HasTarget()))
// 	{
// 		return dllFront->GetBlueFieldPosition();
// 	}
// 	else if ((dllBack != nullptr) && (dllBack->HasTarget()))
// 	{
// 		return dllBack->GetBlueFieldPosition();
// 	}
// 	else
// 	{
// 		return frc::Pose2d{};
// 	}
// }

// frc::Pose2d DragonVision::GetRobotPosition(CAMERA_POSITION position) const
// {
// 	DragonLimelight *limelight = getLimelight(position);
// 	// frc::DriverStation::Alliance alliance = FMSData::GetInstance()->GetAllianceColor();

// 	if ((limelight != nullptr) && (limelight->HasTarget()))
// 	{
// 		return limelight->GetBlueFieldPosition();
// 	}
// 	else
// 	{
// 		return frc::Pose2d{};
// 	}
// }

// /// @brief Gets a pointer to the limelight at the specified position
// /// @param position The physical location of the limelight
// /// @return A pointer to the lilelight object
// DragonLimelight *DragonVision::getLimelight(CAMERA_POSITION position) const
// {
// 	auto theLimeLightInfo = m_DragonLimelightMap.find(position);

// 	if (theLimeLightInfo != m_DragonLimelightMap.end())
// 	{
// 		return theLimeLightInfo->second;
// 	}

// 	return nullptr;
//}