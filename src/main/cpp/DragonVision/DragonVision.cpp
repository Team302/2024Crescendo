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
#include "DragonVision/DragonLimelight.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVisionStructLogger.h"
#include "utils/logging/Logger.h"
#include "utils/DragonField.h"
#include <string>
// Third Party Includes
#include "Limelight/LimelightHelpers.h"

DragonVision *DragonVision::m_dragonVision = nullptr;
DragonVision *DragonVision::GetDragonVision()
{
	if (DragonVision::m_dragonVision == nullptr)
	{
		DragonVision::m_dragonVision = new DragonVision();
	}
	return DragonVision::m_dragonVision;
}

bool DragonVision::HealthCheck(RobotElementNames::CAMERA_USAGE position)
{
	auto camera = m_dragonCameraMap[position];
	return camera->HealthCheck();
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
	if ((position == RobotElementNames::CAMERA_USAGE::LAUNCHE) || (position == RobotElementNames::CAMERA_USAGE::PLACER))
	{
		// may switch above pose fallback to reference or last pose, may be able to use odometry as a fallback and get closest
		// to odometry's slightly inaccurate pose
		m_poseEstimators.emplace_back(photon::PhotonPoseEstimator{GetAprilTagLayout(),
																  photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
																  std::move(photon::PhotonCamera{camera->GetCameraName()}),
																  camera->GetTransformFromRobotCenter()});
		m_poseEstimators.back().SetMultiTagFallbackStrategy(photon::PoseStrategy::AVERAGE_BEST_TARGETS);
	}
}

std::optional<VisionData> DragonVision::GetVisionData(VISION_ELEMENT element)
{
	if ((element == VISION_ELEMENT::NOTE) || (element == VISION_ELEMENT::LAUNCHER_NOTE) || (element == VISION_ELEMENT::PLACER_NOTE)) // if we want to detect a note
	{
		return GetVisionDataFromNote(element);
	}
	else if (element == VISION_ELEMENT::NEAREST_APRILTAG) // nearest april tag
	{
		return GetVisionDataToNearestTag();
	}
	else if ((element == VISION_ELEMENT::STAGE) || (element == VISION_ELEMENT::CENTER_STAGE) || (element == VISION_ELEMENT::LEFT_STAGE) || (element == VISION_ELEMENT::RIGHT_STAGE))
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
	std::optional<VisionData> placerData = std::nullopt;

	if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER] != nullptr)
	{
		placerData = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetDataToNearestAprilTag();
	}

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
	default:
		return std::nullopt;
		break;
	}

	if (placerData)
	{
		if (std::find(tagIdsToCheck.begin(), tagIdsToCheck.end(), placerData.value().tagId) != tagIdsToCheck.end())
		{
			return placerData;
		}
	}

	// tag doesnt matter or no tag
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataToNearestTag()
{
	std::optional<VisionData> selectedData = std::nullopt;
	std::optional<VisionData> launcherData = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->GetDataToNearestAprilTag();
	std::optional<VisionData> placerData = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]->GetDataToNearestAprilTag();

	if ((!launcherData) && (!placerData)) // if we see no april tags
	{
		return std::nullopt;
	}
	else if ((launcherData) && (placerData)) // if we see april tags in both cameras
	{
		// distance logic
		units::length::inch_t launcherDistance = launcherData.value().translationToTarget.X();
		units::length::inch_t placerDistance = placerData.value().translationToTarget.X();

		selectedData = units::math::abs(launcherDistance) <= units::math::abs(placerDistance) ? launcherData : placerData; // if launcher is less ambiguous, select it, and vice versa
	}
	else // one camera sees an april tag
	{
		if (launcherData)
		{

			selectedData = launcherData;
		}
		else if (placerData)
		{
			selectedData = placerData;
		}
	}

	if (selectedData)
	{
		return selectedData;
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetDataToNearestAprilTag(RobotElementNames::CAMERA_USAGE position)
{
	std::optional<VisionData> dataToAprilTag = m_dragonCameraMap[position]->GetDataToNearestAprilTag();
	if ((m_dragonCameraMap[position] != nullptr) && dataToAprilTag.has_value())
	{
		return dataToAprilTag;
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataFromNote(VISION_ELEMENT element)
{
	DragonCamera *selectedCam = nullptr;

	switch (element)
	{
	case VISION_ELEMENT::PLACER_NOTE:
		selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE];
		break;
	case VISION_ELEMENT::LAUNCHER_NOTE:
		selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE];
		break;
	case VISION_ELEMENT::NOTE:
	{

		bool lintakeHasDetection = false;
		bool pintakeHasDetection = false;
		// make sure cameras are set
		if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE] != nullptr)
		{
			lintakeHasDetection = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE]->HasTarget();
		}
		if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE] != nullptr)
		{
			pintakeHasDetection = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE]->HasTarget();
		}

		if (!lintakeHasDetection && !pintakeHasDetection)
		{
			return std::nullopt;
		}
		else if (lintakeHasDetection && pintakeHasDetection)
		{
			// check which note is closest to robot.. and handle std optional
			units::length::meter_t lintakeXDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE]->EstimateTargetXDistance_RelToRobotCoords().value();
			units::length::meter_t lintakeYDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE]->EstimateTargetYDistance_RelToRobotCoords().value();
			frc::Translation2d translationLauncher = frc::Translation2d(lintakeXDistance, lintakeYDistance);

			units::length::meter_t pintakeXDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE]->EstimateTargetXDistance_RelToRobotCoords().value();
			units::length::meter_t pintakeYDistance = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE]->EstimateTargetYDistance_RelToRobotCoords().value();
			frc::Translation2d translationPlacer = frc::Translation2d(pintakeXDistance, pintakeYDistance);

			selectedCam = units::math::abs(translationLauncher.Norm()) < units::math::abs(translationPlacer.Norm()) ? m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE] : m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE];
		}
		else
		{
			if (lintakeHasDetection)
				selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LINTAKE];
			else if (pintakeHasDetection)
				selectedCam = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PINTAKE];
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
		if (!selectedCam->HealthCheck())
		{
			return std::nullopt;
		}

		// create translation using 3 estimated distances
		if (selectedCam->EstimateTargetXDistance_RelToRobotCoords().has_value() || selectedCam->EstimateTargetZDistance_RelToRobotCoords().has_value() || selectedCam->EstimateTargetYDistance_RelToRobotCoords().has_value())
		{
			frc::Translation3d translationToNote = frc::Translation3d(selectedCam->EstimateTargetXDistance_RelToRobotCoords().value(), selectedCam->EstimateTargetYDistance_RelToRobotCoords().value(), selectedCam->EstimateTargetZDistance_RelToRobotCoords().value());
			frc::Rotation3d rotationToNote = frc::Rotation3d();
			// create rotation3d with pitch and yaw (don't have access to roll)
			rotationToNote = frc::Rotation3d(units::angle::degree_t(0.0), selectedCam->GetTargetPitchRobotFrame().value(), selectedCam->GetTargetYawRobotFrame().value());

			// return VisionData with new translation and rotation
			return VisionData{frc::Transform3d(translationToNote, rotationToNote), translationToNote, rotationToNote, -1};
		}
	}
	// if we don't have a selected cam
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataFromElement(VISION_ELEMENT element)
{
	frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();

	// initialize selected field element to empty Pose3d
	frc::Pose3d fieldElementPose = frc::Pose3d{};
	int idToSearch = -1;
	switch (element)
	{
	case VISION_ELEMENT::SPEAKER:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_SPEAKER)} /*load red speaker*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_SPEAKER)}; /*load blue speaker*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 4 : 7;
		break;
	case VISION_ELEMENT::AMP:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_AMP)} /*load red amp*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_AMP)}; /*load blue amp*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 5 : 6;
		break;
	case VISION_ELEMENT::SOURCE:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_SOURCE)} /*load red source*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_SOURCE)}; /*load blue source*/

		break;
	default:
		return std::nullopt;
		break;
	}

	// std::optional<VisionData> multiTagEstimate = MultiTagToElement(fieldElementPose);
	// if (multiTagEstimate)
	// {
	// 	return multiTagEstimate;
	// }

	std::optional<VisionData> singleTagEstimate = SingleTagToElement(fieldElementPose, idToSearch);
	if (singleTagEstimate)
	{
		return singleTagEstimate;
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::MultiTagToElement(frc::Pose3d elementPose)
{
	std::optional<VisionPose> launcherMultiTag = std::nullopt;
	DragonPhotonCam *launcherPhotonCam = dynamic_cast<DragonPhotonCam *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHER]);
	if (launcherPhotonCam != nullptr)
	{
		launcherMultiTag = launcherPhotonCam->GetMultiTagEstimate();
	}

	std::optional<VisionPose> placerMultiTag = std::nullopt;
	DragonPhotonCam *placerPhotonCam = dynamic_cast<DragonPhotonCam *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::PLACER]);
	if (placerPhotonCam != nullptr)
	{
		placerMultiTag = placerPhotonCam->GetMultiTagEstimate();
	}

	std::optional<VisionPose> selectedPose = std::nullopt;

	if (launcherMultiTag && placerMultiTag)
	{
		double launcherAmbiguity = launcherMultiTag.value().visionMeasurementStdDevs[0];
		double placerAmbiguity = placerMultiTag.value().visionMeasurementStdDevs[0];

		selectedPose = launcherAmbiguity <= placerAmbiguity ? launcherMultiTag.value() : placerMultiTag.value(); // if launcher is less ambiguous, select it, and vice versa
	}
	else if (!launcherMultiTag && !placerMultiTag)
	{
		return std::nullopt;
	}
	else
	{
		if (launcherMultiTag)
		{
			selectedPose = launcherMultiTag.value();
		}

		else if (placerMultiTag)
		{
			selectedPose = placerMultiTag.value();
		}
	}

	if (selectedPose)
	{
		// calculate transform to fieldElement as difference between robot pose and field element pose
		frc::Transform3d transformToElement = frc::Transform3d{selectedPose.value().estimatedPose, elementPose};

		// calculate rotation3d for angles from robot center, not transformation
		units::angle::radian_t pitch = units::math::atan2(transformToElement.Z(), transformToElement.X());
		units::angle::radian_t yaw = units::math::atan2(transformToElement.Y(), transformToElement.X());
		units::angle::radian_t roll = units::math::atan2(transformToElement.Z(), transformToElement.Y());
		frc::Rotation3d rotation = frc::Rotation3d(roll, pitch, yaw);

		// rebundle into vision data with april tag thats used
		return VisionData{transformToElement, transformToElement.Translation(), rotation, -1};
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::SingleTagToElement(frc::Pose3d elementPose, int idToSearch)
{
	std::optional<VisionData> launcherAprilTagData = std::nullopt;
	// std::optional<VisionData> selectedData = std::nullopt;

	if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE] != nullptr)
	{
		if (!m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->HealthCheck())
		{
			return std::nullopt;
		}

		// get the optional of the translation and rotation to the apriltag
		launcherAprilTagData = m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->GetDataToSpecifiedTag(idToSearch);
	}

	if (launcherAprilTagData)
	{
		return launcherAprilTagData;

		/* optional of the April Tag's 3D pose
		std::optional<frc::Pose3d> optionalAprilTagPose = GetAprilTagLayout().GetTagPose(selectedData.value().tagId);

		// get valid value of optionalAprilTagPose
		if (optionalAprilTagPose)
		{
			// get the actual pose of the april tag from the optional
			frc::Pose3d aprilTagPose = optionalAprilTagPose.value();

			// get translation and rotation from visiondata
			frc::Transform3d transformToAprilTag = selectedData.value().transformToTarget;

			// translate from apriltag to robot to get robot field position
			frc::Pose3d robotPose = aprilTagPose + transformToAprilTag.Inverse();

			// create transformation from robot to field element
			frc::Transform3d transformToElement = frc::Transform3d(robotPose, elementPose);

			// rebundle into vision data with april tag thats used
			std::optional<VisionData>
				visionData = VisionData(transformToElement,
										transformToElement.Translation(),
										selectedData.value().rotationToTarget, // roll is 0, pitch and yaw are calculated
										selectedData.value().tagId);
			return visionData;
		}
		*/
	}

	return std::nullopt;
}

std::optional<VisionPose> DragonVision::GetRobotPosition()
{
	std::vector<VisionPose> estimatedPoses = {};

	if (!m_photonVisionOdometry)
	{

		// this is for single camera limelight odometry
		if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE] != nullptr)
		{
			if (!m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->HealthCheck())
			{
				return std::nullopt;
			}

			// get the pose from limelight
			DragonLimelight *launcheLimelightCam = dynamic_cast<DragonLimelight *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]);
			std::optional<VisionPose> estimatedPose = launcheLimelightCam->EstimatePoseOdometryLimelight(false); // false since megatag1
			return estimatedPose;
		}
	}
	else
	{
		// this could be moved into photoncamera later
		for (photon::PhotonPoseEstimator estimator : m_poseEstimators)
		{
			units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
			std::optional<photon::EstimatedRobotPose> estimation = estimator.Update();

			if (estimation) // check if we have a result
			{
				frc::Pose3d estimatedPose = estimation.value().estimatedPose;
				units::millisecond_t timestamp = estimation.value().timestamp;

				double ambiguity = 0.0;
				double counter = 0.0;
				wpi::array<double, 3> visionStdMeasurements = VisionPose{}.visionMeasurementStdDevs;

				for (auto target : estimation.value().targetsUsed)
				{
					ambiguity += target.GetPoseAmbiguity();
					counter++;
				}

				// get the average ambiguity and add as standard deviation
				visionStdMeasurements[0] += (ambiguity / counter);
				visionStdMeasurements[1] += (ambiguity / counter);
				visionStdMeasurements[2] += (ambiguity / counter);

				estimatedPoses.emplace_back(VisionPose{estimatedPose, currentTime - timestamp, visionStdMeasurements});
			}
		}

		if (!estimatedPoses.empty())
		{
			if (estimatedPoses.size() == 1)
				return estimatedPoses[0];
			else
			{
				double firstAmbiguity = estimatedPoses[0].visionMeasurementStdDevs[0];
				double secondAmbiguity = estimatedPoses[1].visionMeasurementStdDevs[0];

				return firstAmbiguity < secondAmbiguity ? estimatedPoses[0] : estimatedPoses[1];
			}
		}
	}

	// if we aren't able to calculate our pose from vision, return a null optional
	return std::nullopt;
}

std::optional<VisionPose> DragonVision::GetRobotPositionMegaTag2(units::angle::degree_t yaw,
																 units::angular_velocity::degrees_per_second_t yawRate,
																 units::angle::degree_t pitch,
																 units::angular_velocity::degrees_per_second_t pitchRate,
																 units::angle::degree_t roll,
																 units::angular_velocity::degrees_per_second_t rollRate)
{
	// this is for single camera limelight odometry
	if (m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE] != nullptr)
	{
		if (!m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->HealthCheck())
		{
			return std::nullopt;
		}

		// get the pose from limelight
		LimelightHelpers::SetRobotOrientation(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]->GetCameraName(),
											  yaw.value(),
											  0, 0, 0, 0, 0);

		DragonLimelight *launcheLimelightCam = dynamic_cast<DragonLimelight *>(m_dragonCameraMap[RobotElementNames::CAMERA_USAGE::LAUNCHE]);
		std::optional<VisionPose> estimatedPose = launcheLimelightCam->EstimatePoseOdometryLimelight(true); // true since megatag2
		return estimatedPose;
	}
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

/*****************
 * testAndLogVisionData:  Test and log the vision data
 * add this line to teleopPeriodic to test and log vision data
 *
 *     DragonVision::GetDragonVision()->testAndLogVisionData();
 */
void DragonVision::testAndLogVisionData()
{
	try
	{
		std::optional<VisionPose> visionPose = GetRobotPosition();

		DragonField::GetInstance()->UpdateObjectVisionPose("VisionPose", visionPose);
	}
	catch (std::bad_optional_access &boa)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("testAndLogVisionData"), std::string("bad_optional_access"), boa.what());
	}
	catch (std::exception &e)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("testAndLogVisionData"), std::string("exception"), e.what());
	}
}