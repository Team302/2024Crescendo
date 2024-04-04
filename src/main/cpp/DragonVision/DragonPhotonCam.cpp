//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
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

// Photon Includes
#include "photon/targeting/PhotonPipelineResult.h"
#include "photon/PhotonUtils.h"

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/Timer.h"
#include "frc/geometry/Translation3d.h"

// Team 302 Includes
#include "DragonVision/DragonPhotonCam.h"
#include "DragonVision/DragonVision.h"
#include "DragonVision/DragonPhotonCalculator.h"

DragonPhotonCam::DragonPhotonCam(std::string name,
                                 DragonCamera::PIPELINE initialPipeline,
                                 units::length::inch_t mountingXOffset,
                                 units::length::inch_t mountingYOffset,
                                 units::length::inch_t mountingZOffset,
                                 units::angle::degree_t pitch,
                                 units::angle::degree_t yaw,
                                 units::angle::degree_t roll) : DragonCamera(name, initialPipeline, mountingXOffset, mountingYOffset, mountingZOffset, pitch, yaw, roll),
                                                                m_camera(new photon::PhotonCamera(std::string_view(name.c_str())))

{
    SetPipeline(initialPipeline);
}

bool DragonPhotonCam::HealthCheck()
{
    return false;
}

bool DragonPhotonCam::HasTarget()
{
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return result.HasTargets();
}
std::optional<VisionPose> DragonPhotonCam::GetFieldPosition()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check if we have detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // Get transformation from camera to target
        frc::Transform3d camToTargetTransform = target.GetBestCameraToTarget();

        // get detected tag id
        int tagId = target.GetFiducialId();

        std::optional<frc::Pose3d> potentialPose = DragonVision::GetAprilTagLayout().GetTagPose(tagId);

        if (potentialPose.has_value())
        {
            frc::Pose3d fieldRelativeTagPose = *potentialPose;

            // Initial pose = Tag (field relative)
            // Transform target -> cam (reason for inverse is we get cam -> target, direction matters)
            // Then transform cam -> robot (again, need to inverse since we get robot -> cam)
            // Final pose = Robot (field relative)
            frc::Pose3d fieldRelPose = fieldRelativeTagPose + camToTargetTransform.Inverse() + m_robotCenterToCam.Inverse();

            units::time::millisecond_t timestamp = frc::Timer::GetFPGATimestamp() - result.GetLatency();

            // Get the pose ambiguity from PhotonVision
            double ambiguity = target.GetPoseAmbiguity();

            // Get the default values for std deviations
            wpi::array<double, 3> visionStdMeasurements = VisionPose{}.visionMeasurementStdDevs;

            // Add ambiguity to those default values, may want to multiply by some factor to decrease our confidence depending on ambiguity
            visionStdMeasurements[0] += ambiguity;
            visionStdMeasurements[1] += ambiguity;
            visionStdMeasurements[2] += ambiguity;

            return VisionPose(fieldRelPose, timestamp, visionStdMeasurements, PoseEstimationStrategy::SINGLE_TAG);
        }
    }

    return std::nullopt;
}

std::optional<VisionPose> DragonPhotonCam::GetFieldPosition(frc::DriverStation::Alliance alliance)
{
    return GetFieldPosition();
}

std::optional<VisionPose> DragonPhotonCam::GetMultiTagEstimate()
{
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    if (result.MultiTagResult().result.isPresent)
    {
        // field to camera transform
        frc::Transform3d transform = result.MultiTagResult().result.best;

        frc::Pose3d robotPose = frc::Pose3d{} + (transform + m_robotCenterToCam.Inverse());

        units::time::millisecond_t timestamp = frc::Timer::GetFPGATimestamp() - result.GetLatency();

        // Get the default values for std deviations
        wpi::array<double, 3> visionStdMeasurements = VisionPose{}.visionMeasurementStdDevs;

        // Get the pose ambiguity from PhotonVision
        double ambiguity = result.MultiTagResult().result.ambiguity;

        // Add ambiguity to those default values, may want to multiply by some factor to decrease our confidence depending on ambiguity
        visionStdMeasurements[0] += ambiguity;
        visionStdMeasurements[1] += ambiguity;
        visionStdMeasurements[2] += ambiguity;

        return VisionPose(robotPose, timestamp, visionStdMeasurements, PoseEstimationStrategy::MULTI_TAG);
    }

    return std::nullopt;
}

double DragonPhotonCam::GetPoseAmbiguity()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check if we have detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        return target.GetPoseAmbiguity();
    }

    return 9999.0; // if we don't have targets, return very large ambiguity 999999 - may want to make this optional
}

/// @brief  get Yaw of possible target.
/// @return units::angle::degree_t - Counter Clockwise/left for positive.
std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetYaw()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check if we have detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // return Yaw. This also multiplys it by -1 to invert to a global use in the 302 code
        return units::angle::degree_t(-1.0 * target.GetYaw());
    }

    return std::nullopt;
}

/// @brief get TargetSkew of possible target.
/// @return Skew casted as units::angle::degree_t. Counter Clockwise/left for positive.
std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetSkew()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check if we have detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // return degree skew as units object. This would originally be a double
        return units::angle::degree_t(target.GetSkew());
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetYawRobotFrame()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return DragonPhotonCalculator::GetTargetYawRobotFrame(m_cameraPose, result);
}

std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetPitchRobotFrame()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return DragonPhotonCalculator::GetTargetPitchRobotFrame(m_cameraPose, result);
}

/// @brief Get Pitch to Target
/// @return units::angle::degree_t - positive up
std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetPitch()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // return
        return units::angle::degree_t(target.GetPitch());
    }

    return std::nullopt;
}

std::optional<units::time::millisecond_t> DragonPhotonCam::GetPipelineLatency()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // get the total latency
    units::second_t latency = result.GetLatency();

    return latency;
}

std::optional<int> DragonPhotonCam::GetAprilTagID()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check if we have detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // return detected tag's id
        return target.GetFiducialId();
    }

    return std::nullopt;
}

/// @brief Get target area
/// @return Double - Percentage (0-100)
std::optional<double> DragonPhotonCam::GetTargetArea()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // return
        return target.GetArea();
    }

    return std::nullopt;
}

/// @brief Estimate the X distance to the detected target
/// @return units::length::inch_t - Positive is forward
std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetXDistance()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();
        return transform.X();
    }

    return std::nullopt;
}

/// @brief Estimate the Y distance to the detected target
/// @return units::length::inch_t - Positive is left
std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetYDistance()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();

        return transform.Y();
    }

    return std::nullopt;
}

/// @brief Estimate the Z distance to the detected target
/// @return units::length::inch_t - Positive is up
std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetZDistance()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {

        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();

        return transform.Z();
    }

    return std::nullopt;
}

/// @brief Estimate the X distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is forward

std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetXDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return DragonPhotonCalculator::EstimateTargetXDistance_RelToRobotCoords(m_robotCenterToCam, result);
}

/// @brief Estimate the Y distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is left
std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetYDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return DragonPhotonCalculator::EstimateTargetYDistance_RelToRobotCoords(m_robotCenterToCam, result);
}

/// @brief Estimate the Z distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is up

std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetZDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return DragonPhotonCalculator::EstimateTargetZDistance_RelToRobotCoords(m_robotCenterToCam, result);
}

bool DragonPhotonCam::UpdatePipeline()
{
    m_camera->SetPipelineIndex(static_cast<int>(m_pipeline));
    return false;
}

std::optional<VisionData> DragonPhotonCam::GetDataToNearestAprilTag()
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    // result.GetTargets().back().

    return DragonPhotonCalculator::GetDataToNearestAprilTag(m_cameraPose, result);
}

std::optional<VisionData> DragonPhotonCam::GetDataToSpecifiedTag(int id)
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    auto targets = result.GetTargets();
    for (photon::PhotonTrackedTarget target : targets)
    {
        if (target.fiducialId == id)
        {
            return DragonPhotonCalculator::GetDataToNearestAprilTag(m_cameraPose, target);
        }
    }

    return std::nullopt;
}