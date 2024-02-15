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

            units::time::millisecond_t timestamp = frc::Timer::GetFPGATimestamp();

            // Get the pose ambiguity from PhotonVision
            double ambiguity = target.GetPoseAmbiguity();

            // Get the default values for std deviations
            wpi::array<double, 3> visionStdMeasurements = VisionPose{}.visionMeasurementStdDevs;

            // Add ambiguity to those default values, may want to multiply by some factor to decrease our confidence depending on ambiguity
            visionStdMeasurements[0] += ambiguity;
            visionStdMeasurements[1] += ambiguity;
            visionStdMeasurements[2] += ambiguity;

            return std::make_optional(VisionPose(fieldRelPose, timestamp, visionStdMeasurements));
        }
    }

    return std::nullopt;
}

std::optional<VisionPose> DragonPhotonCam::GetFieldPosition(frc::DriverStation::Alliance alliance)
{
    return GetFieldPosition();
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
    else
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
    else
        return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetYawRobotFrame()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        int tagId = target.GetFiducialId();

        // if we don't see a tag we are detecting a note instead
        if (tagId == -1)
        {
            // Use photon utils to calculate distance to target
            units::meter_t distanceToTarget = photon::PhotonUtils::CalculateDistanceToTarget(m_cameraPose.Z(),                    // camera mounting height
                                                                                             m_noteVerticalOffset,                // offset from robot center based on note being on the floor
                                                                                             m_cameraPose.Rotation().Y(),         // camera mounting pitch
                                                                                             units::degree_t{target.GetPitch()}); // pitch of detection

            // get y distance to camera by multiplying distanceToTarget(x distance/adjacent) to the tangent of the yaw (opposite = ydistance, adjacent = x distance)
            units::length::meter_t yOffsetCamToTarget = distanceToTarget * units::math::tan(units::degree_t{target.GetYaw()});

            // Add camera y offset from robot center to target's y offset from cam
            units::length::meter_t yOffsetRobotToTarget = yOffsetCamToTarget + m_cameraPose.Y();

            // inverse tangent of opposite (y/ left/right offset) over adjacent (x distance)
            units::angle::radian_t yawRobotRelative = units::math::atan2(yOffsetRobotToTarget, distanceToTarget);

            return yawRobotRelative;
        }
        else // we see an april tag
        {
            // transform to get from cam to target
            frc::Transform3d camToTarget = target.GetBestCameraToTarget();

            // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
            units::angle::radian_t yawRobotRelative = units::math::atan2(frc::Transform3d(frc::Pose3d{}, (m_cameraPose + camToTarget)).Y(), frc::Transform3d(frc::Pose3d{}, (m_cameraPose + camToTarget)).X());

            return yawRobotRelative;
        }
    }

    else
        return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCam::GetTargetPitchRobotFrame()
{
    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        int tagId = target.GetFiducialId();

        // if we don't see a tag we are detecting a note instead
        if (tagId == -1)
        {
            // Use photon utils to calculate distance to target
            units::meter_t distanceToTarget = photon::PhotonUtils::CalculateDistanceToTarget(m_cameraPose.Z(),                    // camera mounting height
                                                                                             m_noteVerticalOffset,                // offset from robot center based on note being on the floor
                                                                                             m_cameraPose.Rotation().Y(),         // camera mounting pitch
                                                                                             units::degree_t{target.GetPitch()}); // pitch of detection

            // inverse tangent of opposite (z/height) over adjacent (x distance)
            units::angle::radian_t pitchRobotRelative = units::math::atan2(m_cameraPose.Z(), distanceToTarget);

            return pitchRobotRelative;
        }
        else // we see an april tag
        {
            // transform to get from cam to target
            frc::Transform3d camToTarget = target.GetBestCameraToTarget();

            // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
            units::angle::radian_t pitchRobotRelative = units::math::atan2(frc::Transform3d(frc::Pose3d{}, (m_cameraPose + camToTarget)).Z(), frc::Transform3d(frc::Pose3d{}, (m_cameraPose + camToTarget)).X());

            return pitchRobotRelative;
        }
    }
    else
        return std::nullopt;
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
    else
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
    else
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
    else
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
    else
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
    else
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

    else
        return std::nullopt;
}

/// @brief Estimate the X distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is forward

std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetXDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().X() + m_robotCenterToCam.X();
    }
    else
        return std::nullopt;
}

/// @brief Estimate the Y distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is left
std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetYDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().Y() + m_robotCenterToCam.Y();
    }
    else
        return std::nullopt;
}

/// @brief Estimate the Z distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is up

std::optional<units::length::inch_t> DragonPhotonCam::EstimateTargetZDistance_RelToRobotCoords()
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // get latest detections
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().Z() + m_robotCenterToCam.Z();
    }
    else
        return std::nullopt;
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

    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        frc::Transform3d camToTargetTransform = target.GetBestCameraToTarget();

        frc::Translation3d translation = frc::Transform3d{frc::Pose3d{}, (m_cameraPose + camToTargetTransform)}.Translation();

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(0.0),        // roll
                                                   GetTargetPitchRobotFrame().value(), // pitch
                                                   GetTargetYawRobotFrame().value()};  // yaw

        return std::make_optional(VisionData{frc::Transform3d(translation, rotation), GetAprilTagID().value()});
    }
    else
        return std::nullopt;
}