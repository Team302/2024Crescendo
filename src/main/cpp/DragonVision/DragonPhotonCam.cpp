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
                                                                m_camera(new photon::PhotonCamera(name))
{
    SetPipeline(initialPipeline);
}

bool DragonPhotonCam::HasTarget() const
{
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    return result.HasTargets();
}
VisionPose DragonPhotonCam::GetFieldPosition()
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

            return VisionPose(fieldRelPose, timestamp, visionStdMeasurements);
        }
    }

    return VisionPose{};
}

VisionPose DragonPhotonCam::GetFieldPosition(frc::DriverStation::Alliance alliance)
{
    return GetFieldPosition();
}

double DragonPhotonCam::GetPoseAmbiguity() const
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
units::angle::degree_t DragonPhotonCam::GetTargetYaw() const
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

    // if no tag found, return 0
    return units::angle::degree_t(0.0);
}

/// @brief get TargetSkew of possible target.
/// @return Skew casted as units::angle::degree_t. Counter Clockwise/left for positive.
units::angle::degree_t DragonPhotonCam::GetTargetSkew() const
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

    // if no tag found, return 0
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonPhotonCam::GetTargetYawRobotFrame() const
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

            // Get the translation component from cam to target
            frc::Translation3d camToTargetTranslation = camToTarget.Translation();

            // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
            units::angle::radian_t yawRobotRelative = units::math::atan2(m_cameraPose.Y() + camToTargetTranslation.Y(), m_cameraPose.X() + camToTargetTranslation.X());

            return yawRobotRelative;
        }
    }

    return units::angle::degree_t(-1.0);
}

units::angle::degree_t DragonPhotonCam::GetTargetPitchRobotFrame() const
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
            // pitch of detection
            units::angle::radian_t cameraPitch = units::degree_t(target.GetPitch());

            // transform to get from cam to target
            frc::Transform3d camToTarget = target.GetBestCameraToTarget();

            // Get the translation component from cam to target
            frc::Translation3d camToTargetTranslation = camToTarget.Translation();

            // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
            units::angle::radian_t pitchRobotRelative = units::math::atan2(m_cameraPose.Z() + camToTargetTranslation.Z(), m_cameraPose.X() + camToTargetTranslation.X());

            return pitchRobotRelative;
        }
    }

    return units::angle::degree_t(-1.0);
}

/// @brief Get Pitch to Target
/// @return units::angle::degree_t - positive up
units::angle::degree_t DragonPhotonCam::GetTargetPitch() const
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

    // if it isn't found
    return units::angle::degree_t(0.0);
}

units::time::millisecond_t DragonPhotonCam::GetPipelineLatency() const
{
    // get latest detections from co-processor
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();

    // get the total latency
    units::second_t latency = result.GetLatency();

    return latency;
}

int DragonPhotonCam::GetAprilTagID() const
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

    // if no tag found, return -1
    return -1;
}

/// @brief Get target area
/// @return Double - Percentage (0-100)
double DragonPhotonCam::GetTargetArea() const
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

    // if it isn't found
    return -1;
}

/// @brief Estimate the X distance to the detected target
/// @return units::length::inch_t - Positive is forward
units::length::inch_t DragonPhotonCam::EstimateTargetXDistance() const
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

    return units::length::inch_t(-1.0);
}

/// @brief Estimate the Y distance to the detected target
/// @return units::length::inch_t - Positive is left
units::length::inch_t DragonPhotonCam::EstimateTargetYDistance() const
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

    return units::length::inch_t(-1.0);
}

/// @brief Estimate the Z distance to the detected target
/// @return units::length::inch_t - Positive is up
units::length::inch_t DragonPhotonCam::EstimateTargetZDistance() const
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

    return units::length::inch_t(-1.0);
}

/// @brief Estimate the X distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is forward

units::length::inch_t DragonPhotonCam::EstimateTargetXDistance_RelToRobotCoords() const
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

    return units::length::inch_t(-1.0);
}

/// @brief Estimate the Y distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is left
units::length::inch_t DragonPhotonCam::EstimateTargetYDistance_RelToRobotCoords() const
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

    return units::length::inch_t(-1.0);
}

/// @brief Estimate the Z distance to the detected target in relation to robot
/// @return units::length::inch_t - Positive is up

units::length::inch_t DragonPhotonCam::EstimateTargetZDistance_RelToRobotCoords() const
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

    return units::length::inch_t(-1.0);
}
bool DragonPhotonCam::UpdatePipeline(DragonCamera::PIPELINE pipeline)
{
    m_camera->SetPipelineIndex(static_cast<int>(m_pipeline));
    return false;
}

VisionData DragonPhotonCam::GetDataToNearestAprilTag()
{
    // get latest detections from co-processor
    frc::Transform3d camToTargetTransform;
    photon::PhotonPipelineResult result = m_camera->GetLatestResult();
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        camToTargetTransform = target.GetBestCameraToTarget();
    }
    return VisionData{camToTargetTransform, GetAprilTagID()};
}