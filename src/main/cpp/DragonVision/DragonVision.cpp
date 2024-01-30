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
#include "DragonVision/DragonPhotonCam.h"
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

frc::AprilTagFieldLayout DragonVision::m_aprilTagLayout = frc::AprilTagFieldLayout();
frc::AprilTagFieldLayout DragonVision::GetAprilTagLayout()
{
	if (DragonVision::m_aprilTagLayout != frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo))
	{
		DragonVision::m_aprilTagLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
	}
	return DragonVision::m_aprilTagLayout;
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
	if (element == VISION_ELEMENT::NOTE || element == VISION_ELEMENT::LAUNCHER_NOTE || element == VISION_ELEMENT::PLACER_NOTE) // need to add LAUNCHER_NOTE AND PLACER_NOTE
	{
		return GetVisionDataFromNote(element);
	}
	else if (element == VISION_ELEMENT::NEAREST_APRILTAG) // nearest april tag
	{
		return GetVisionDataToNearestTag();
	}
	else // looking for april tag elements
	{
		return GetVisionDataFromElement(element);
	}

	// if we don't see any vision targets, return null optional
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataToNearestTag()
{
	DragonCamera *selectedCam = nullptr;

	int frontTagId = m_DragonCameraMap[LAUNCHER]->GetAprilTagID();
	int backTagId = m_DragonCameraMap[PLACER]->GetAprilTagID();

	if ((frontTagId == -1) && (backTagId == -1)) // if we see no april tags
	{
		return std::nullopt;
	}
	else if ((frontTagId != -1) && (backTagId != -1)) // if we see april tags in both cameras
	{
		// distance logic
		units::length::inch_t frontDistance = m_DragonCameraMap[LAUNCHER]->GetEstimatedTargetXDistance_RelToRobotCoords();
		units::length::inch_t backDistance = m_DragonCameraMap[PLACER]->GetEstimatedTargetXDistance_RelToRobotCoords();

		selectedCam = frontDistance <= backDistance ? m_DragonCameraMap[LAUNCHER] : m_DragonCameraMap[PLACER]; // if front is less ambiguous, select it, and vice versa
	}
	else // one camera sees an april tag
	{
		if (frontTagId != -1)
			selectedCam = m_DragonCameraMap[LAUNCHER];
		else
			selectedCam = m_DragonCameraMap[PLACER];
	}

	return selectedCam->GetDataToNearestApriltag();
}

std::optional<VisionData> DragonVision::GetVisionDataFromNote(VISION_ELEMENT element)
{
	DragonCamera *selectedCam = nullptr;

	switch (element)
	{
	case VISION_ELEMENT::PLACER_NOTE:
		selectedCam = m_DragonCameraMap[PLACER];
		break;
	case VISION_ELEMENT::LAUNCHER_NOTE:
		selectedCam = m_DragonCameraMap[LAUNCHER];
		break;
	case VISION_ELEMENT::NOTE:
	{
		bool frontHasDetection = m_DragonCameraMap[LAUNCHER_INTAKE]->HasTarget();
		bool backHasDetection = m_DragonCameraMap[PLACER_INTAKE]->HasTarget();
		if (!frontHasDetection && !backHasDetection)
		{
			return std::nullopt;
		}
		else if (frontHasDetection && backHasDetection)
		{

			frc::Translation2d translationfront = frc::Translation2d(m_DragonCameraMap[LAUNCHER_INTAKE]->GetEstimatedTargetXDistance_RelToRobotCoords(), m_DragonCameraMap[LAUNCHER_INTAKE]->GetEstimatedTargetYDistance_RelToRobotCoords());
			frc::Translation2d translationback = frc::Translation2d(m_DragonCameraMap[PLACER_INTAKE]->GetEstimatedTargetXDistance_RelToRobotCoords(), m_DragonCameraMap[PLACER_INTAKE]->GetEstimatedTargetYDistance_RelToRobotCoords());

			selectedCam = units::math::abs(translationfront.Norm()) < units::math::abs(translationback.Norm()) ? m_DragonCameraMap[LAUNCHER_INTAKE] : m_DragonCameraMap[PLACER_INTAKE];
		}
		else
		{
			if (frontHasDetection)
				selectedCam = m_DragonCameraMap[LAUNCHER_INTAKE];
			else
				selectedCam = m_DragonCameraMap[PLACER_INTAKE];
		}
	}
	break;
	default:
		break;
	}

	// now we have selected camera
	// to get robot relative measurements, use following functions:

	// create translation 3d, create std::optional visiondata with that translation3d
	// return that translation3d
	if (selectedCam != nullptr)
	{
		frc::Translation3d translationToNote = frc::Translation3d(selectedCam->GetEstimatedTargetXDistance_RelToRobotCoords(), selectedCam->GetEstimatedTargetYDistance_RelToRobotCoords(), selectedCam->GetEstimatedTargetZDistance_RelToRobotCoords());
		frc::Rotation3d rotationToNote = frc::Rotation3d(units::angle::degree_t(0.0), selectedCam->GetTargetPitchRobotFrame(), selectedCam->GetTargetYawRobotFrame());
		return std::optional<VisionData>{frc::Transform3d(translationToNote, rotationToNote)};
	}
}

std::optional<VisionData> DragonVision::GetVisionDataFromElement(VISION_ELEMENT element)
{
	DragonCamera *selectedCam = nullptr;

	bool bothCamerasSeeTag = (m_DragonCameraMap[LAUNCHER]->GetAprilTagID() != -1) && (m_DragonCameraMap[PLACER]->GetAprilTagID() != -1);

	int frontTagId = m_DragonCameraMap[LAUNCHER]->GetAprilTagID();
	int backTagId = m_DragonCameraMap[PLACER]->GetAprilTagID();

	if ((frontTagId == -1) && (backTagId == -1)) // if we see no april tags
	{
		return std::nullopt;
	}
	else if ((frontTagId != -1) && (backTagId != -1)) // if we see april tags in both cameras
	{
		// confidence logic
		double frontAmbiguity = dynamic_cast<DragonPhotonCam *>(m_DragonCameraMap[LAUNCHER])->GetPoseAmbiguity();
		double backAmbiguity = dynamic_cast<DragonPhotonCam *>(m_DragonCameraMap[PLACER])->GetPoseAmbiguity();

		selectedCam = frontAmbiguity <= backAmbiguity ? m_DragonCameraMap[LAUNCHER] : m_DragonCameraMap[PLACER]; // if front is less ambiguous, select it, and vice versa
	}
	else // one camera sees an april tag
	{
		if (frontTagId != -1)
			selectedCam = m_DragonCameraMap[LAUNCHER];

		else
			selectedCam = m_DragonCameraMap[PLACER];
	}

	// if (FMSData::GetInstance()->GetAllianceColor()
	frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

	frc::Pose3d fieldElementPose = frc::Pose3d{};

	switch (element)
	{
	case VISION_ELEMENT::SPEAKER:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{} /*load red speaker*/ : frc::Pose3d{}; /*load blue speaker*/
		break;
	case VISION_ELEMENT::AMP:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{} /*load red amp*/ : frc::Pose3d{}; /*load blue amp*/
		break;
	}

	// make 2 pose 3ds and implement in transform3d.
	// https: // github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_transform3d.html#a31810c15a05d3a2a8981462c88d965e4

	// determine color of field element based on alliance color
	// get the field pose of the specified element, will use FieldConstants file that isn't created
	// get pose3d of detected april tag, use VisionData GetDataToNearestAprilTag();
	// get the translation from april tag to field element
	// use measurements from robot to april tag to calculate distances to field element
	// return vision data with translation3d and detected tag
}

std::optional<VisionPose> DragonVision::GetRobotPosition()
{
	// if we aren't able to calculate our pose from vision, return a null optional
	return std::nullopt;
}

bool DragonVision::SetPipeline(DragonCamera::PIPELINE mode, CAMERA_POSITION position)
{
	m_DragonCameraMap[position]->SetPipeline(mode);
	m_DragonCameraMap[position]->UpdatePipeline();
	return false;
}

DragonCamera::PIPELINE DragonVision::GetPipeline(CAMERA_POSITION position)
{
	return m_DragonCameraMap[position]->GetPipeline();
}

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