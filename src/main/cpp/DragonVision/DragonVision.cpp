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
#include <string>

// FRC Includes
#include "frc/Timer.h"

// Team 302 includes

#include "DragonVision/DragonVision.h"
#include "DragonVision/DragonPhotonCam.h"
#include "utils/FMSData.h"

#include <string>
// Third Party Includes

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

void DragonVision::AddCamera(DragonCamera *camera, RobotElementNames::CAMERA_USAGE position)
{
	m_dragonCameraMap[position] = camera;

	// check if we should add camera to photon pose estimator
	if ((position == RobotElementNames::CAMERA_USAGE::LAUNCHER) || (position == RobotElementNames::CAMERA_USAGE::PLACER))
	{
		m_poseEstimators.emplace_back(photon::PhotonPoseEstimator{GetAprilTagLayout(),
																  photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
																  std::move(photon::PhotonCamera{camera->GetCameraName()}),
																  camera->GetTransformFromRobotCenter()});
	}
}

std::optional<VisionData> DragonVision::GetVisionData(VISION_ELEMENT element)
{
	if (element == VISION_ELEMENT::NOTE || element == VISION_ELEMENT::LAUNCHER_NOTE || element == VISION_ELEMENT::PLACER_NOTE) // if we want to detect a note
	{
		return GetVisionDataFromNote(element);
	}
	else if (element == VISION_ELEMENT::NEAREST_APRILTAG) // nearest april tag
	{
		return GetVisionDataToNearestTag();
	}
	else if (element == VISION_ELEMENT::STAGE || element == VISION_ELEMENT::CENTER_STAGE || element == VISION_ELEMENT::LEFT_STAGE || element == VISION_ELEMENT::RIGHT_STAGE)
	{
		return GetVisionDataToNearestStageTag(element);
	}
	else // looking for april tag elements
	{
		return GetVisionDataFromElement(element);
	}

	// if we don't see any vision targets, return null optional
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataToNearestStageTag(VISION_ELEMENT element)
{
	int launcherTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]->GetAprilTagID().value();
	int placerTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetAprilTagID().value();

	// get alliance color from FMSData
	frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

	// initialize tags to check to null pointer
	std::vector<int> tagIdsToCheck = {};
	switch (element)
	{
	case VISION_ELEMENT::STAGE:
		if (allianceColor == frc::DriverStation::Alliance::kBlue)
		{
			// blue alliance stage tag ids are 14, 15, 16
			tagIdsToCheck.emplace_back(14);
			tagIdsToCheck.emplace_back(15);
			tagIdsToCheck.emplace_back(16);
		}
		else
		{
			// red alliance stage tag ids are 11, 12, 13
			tagIdsToCheck.emplace_back(11);
			tagIdsToCheck.emplace_back(12);
			tagIdsToCheck.emplace_back(13);
		}
		break;
	case VISION_ELEMENT::LEFT_STAGE:
		if (allianceColor == frc::DriverStation::Alliance::kBlue)
		{
			tagIdsToCheck.emplace_back(15);
		}
		else
		{
			tagIdsToCheck.emplace_back(11);
		}
		break;
	case VISION_ELEMENT::RIGHT_STAGE:
		if (allianceColor == frc::DriverStation::Alliance::kBlue)
		{
			tagIdsToCheck.emplace_back(16);
		}
		else
		{
			tagIdsToCheck.emplace_back(12);
		}
		break;
	case VISION_ELEMENT::CENTER_STAGE:
		if (allianceColor == frc::DriverStation::Alliance::kBlue)
		{
			tagIdsToCheck.emplace_back(14);
		}
		else
		{
			tagIdsToCheck.emplace_back(13);
		}
		break;
	}
	if (std::find(tagIdsToCheck.begin(), tagIdsToCheck.end(), launcherTagId) != tagIdsToCheck.end())
	{
		return m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]->GetDataToNearestAprilTag().value(); // launcherTagId is for stage id
	}
	else if (std::find(tagIdsToCheck.begin(), tagIdsToCheck.end(), placerTagId) != tagIdsToCheck.end())
	{
		return m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetDataToNearestAprilTag().value(); // placerTagId is for stage id
	}

	// tag doesnt matter or no tag
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataToNearestTag()
{
	DragonCamera *selectedCam = nullptr;
	int launcherTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]->GetAprilTagID().value();
	int placerTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetAprilTagID().value();

	if ((!launcherTagId) && (!placerTagId)) // if we see no april tags
	{
		return std::nullopt;
	}
	else if ((launcherTagId) && (placerTagId)) // if we see april tags in both cameras
	{
		// distance logic
		units::length::inch_t launcherDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]->EstimateTargetXDistance_RelToRobotCoords().value();
		units::length::inch_t placerDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->EstimateTargetXDistance_RelToRobotCoords().value();

		selectedCam = units::math::abs(launcherDistance) <= units::math::abs(placerDistance) ? m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER] : m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]; // if front is less ambiguous, select it, and vice versa
	}
	else // one camera sees an april tag
	{
		if (launcherTagId)
			selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER];
		else
			selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER];
	}

	if (selectedCam != nullptr)
	{
		return selectedCam->GetDataToNearestAprilTag().value();
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetDataToNearestAprilTag(RobotElementNames::CAMERA_USAGE position)
{
	if (m_dragonCameraMap[position] != nullptr)
	{

		return m_dragonCameraMap[position]->GetDataToNearestAprilTag().value();
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataFromNote(VISION_ELEMENT element)
{
	DragonCamera *selectedCam = nullptr;

	switch (element)
	{
	case VISION_ELEMENT::PLACER_NOTE:
		selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER];
		break;
	case VISION_ELEMENT::LAUNCHER_NOTE:
		selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER];
		break;
	case VISION_ELEMENT::NOTE:
	{
		bool frontHasDetection = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER_INTAKE]->HasTarget();
		bool backHasDetection = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER_INTAKE]->HasTarget();
		if (!frontHasDetection && !backHasDetection)
		{
			return std::nullopt;
		}
		else if (frontHasDetection && backHasDetection)
		{
			// check which note is closest to robot
			frc::Translation2d translationLauncher = frc::Translation2d(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER_INTAKE]->EstimateTargetXDistance_RelToRobotCoords().value(), m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER_INTAKE]->EstimateTargetYDistance_RelToRobotCoords().value());
			frc::Translation2d translationPlacer = frc::Translation2d(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER_INTAKE]->EstimateTargetXDistance_RelToRobotCoords().value(), m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER_INTAKE]->EstimateTargetYDistance_RelToRobotCoords().value());

			selectedCam = units::math::abs(translationLauncher.Norm()) < units::math::abs(translationPlacer.Norm()) ? m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER_INTAKE] : m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER_INTAKE];
		}
		else
		{
			if (frontHasDetection)
				selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER_INTAKE];
			else
				selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER_INTAKE];
		}
	}
	break;
	default:
		// no-op
		break;
	}

	// double check selectedCam is not nullptr
	if (selectedCam != nullptr)
	{
		// create translation using 3 estimated distances
		frc::Translation3d translationToNote = frc::Translation3d(selectedCam->EstimateTargetXDistance_RelToRobotCoords().value(), selectedCam->EstimateTargetYDistance_RelToRobotCoords().value(), selectedCam->EstimateTargetZDistance_RelToRobotCoords().value());

		// create rotation3d with pitch and yaw (don't have access to roll)
		frc::Rotation3d rotationToNote = frc::Rotation3d(units::angle::degree_t(0.0), selectedCam->GetTargetPitchRobotFrame().value(), selectedCam->GetTargetYawRobotFrame().value());

		// return VisionData with new translation and rotation
		return std::optional<VisionData>{frc::Transform3d(translationToNote, rotationToNote)};
	}

	// if we don't have a selected cam
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataFromElement(VISION_ELEMENT element)
{
	DragonCamera *selectedCam = nullptr;

	int launcherTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]->GetAprilTagID().value();
	int placerTagId = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetAprilTagID().value();

	if ((!launcherTagId) && (!placerTagId)) // if we see no april tags
	{
		return std::nullopt;
	}
	else if ((launcherTagId) && (placerTagId)) // if we see april tags in both cameras
	{
		// confidence logic
		double launcherAmbiguity = dynamic_cast<DragonPhotonCam *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER])->GetPoseAmbiguity();
		double placerAmbiguity = dynamic_cast<DragonPhotonCam *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER])->GetPoseAmbiguity();

		selectedCam = launcherAmbiguity <= placerAmbiguity ? m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER] : m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]; // if launcher is less ambiguous, select it, and vice versa
	}
	else // one camera sees an april tag
	{
		if (launcherTagId)
			selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER];

		else
			selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER];
	}

	frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

	// initialize selected field element to empty Pose3d
	frc::Pose3d fieldElementPose = frc::Pose3d{};
	switch (element)
	{
	case VISION_ELEMENT::SPEAKER:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{} /*load red speaker*/ : frc::Pose3d{}; /*load blue speaker*/
		break;
	case VISION_ELEMENT::AMP:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{} /*load red amp*/ : frc::Pose3d{}; /*load blue amp*/
		break;
	case VISION_ELEMENT::SOURCE:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{} /*load red source*/ : frc::Pose3d{}; /*load blue source*/
		break;
	default:
		// no-op
		break;
	}

	// optional of the April Tag's 3D pose
	std::optional<frc::Pose3d> optionalAprilTagPose = GetAprilTagLayout().GetTagPose(selectedCam->GetAprilTagID().value());

	// get valid value of optionalAprilTagPose
	if (optionalAprilTagPose)
	{
		frc::Pose3d AprilTagPose = optionalAprilTagPose.value();
		std::optional<VisionData> dataToAprilTag = selectedCam->GetDataToNearestAprilTag();
		frc::Transform3d transformToAprilTag = dataToAprilTag.value().deltaToTarget;
		frc::Pose3d robotPose = AprilTagPose + transformToAprilTag.Inverse();
		frc::Transform3d transformToElement = frc::Transform3d(robotPose, fieldElementPose);
		std::optional<VisionData> visionData = VisionData(transformToElement, selectedCam->GetAprilTagID().value());
		return visionData;
	}

	return std::nullopt;

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
	std::vector<VisionPose> estimatedPoses = {};

	for (photon::PhotonPoseEstimator estimator : m_poseEstimators)
	{
		units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
		std::optional<photon::EstimatedRobotPose> estimation = estimator.Update();

		if (estimation) // check if we have a result
		{
			frc::Pose3d estimatedPose = estimation.value().estimatedPose;
			units::millisecond_t timestamp = estimation.value().timestamp;

			// returned result

			double ambiguity = 0.0;
			wpi::array<double, 3> visionStdMeasurements = VisionPose{}.visionMeasurementStdDevs;

			for (auto target : estimation.value().targetsUsed)
			{
				ambiguity += target.GetPoseAmbiguity();
				visionStdMeasurements[0] += ambiguity;
				visionStdMeasurements[1] += ambiguity;
				visionStdMeasurements[2] += ambiguity;
			}

			estimatedPoses.emplace_back(VisionPose{estimatedPose, currentTime - timestamp, visionStdMeasurements});
		}
	}

	if (!estimatedPoses.empty())
	{
		if (estimatedPoses.size() == 1)
			return std::make_optional(estimatedPoses[0]);
		else
		{
			double firstAmbiguity = estimatedPoses[0].visionMeasurementStdDevs[0];
			double secondAmbiguity = estimatedPoses[1].visionMeasurementStdDevs[0];

			return firstAmbiguity < secondAmbiguity ? std::make_optional(estimatedPoses[0]) : std::make_optional(estimatedPoses[1]);
		}
	}

	// if we aren't able to calculate our pose from vision, return a null optional
	return std::nullopt;
}

bool DragonVision::SetPipeline(DragonCamera::PIPELINE mode, RobotElementNames::CAMERA_USAGE position)
{
	m_dragonCameraMap[position]->SetPipeline(mode);
	m_dragonCameraMap[position]->UpdatePipeline();
	return false;
}

DragonCamera::PIPELINE DragonVision::GetPipeline(RobotElementNames::CAMERA_USAGE position)
{
	return m_dragonCameraMap[position]->GetPipeline();
}