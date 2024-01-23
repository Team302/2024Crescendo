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

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"

// Team 302 Includes
#include "DragonVision/DragonPhotonCam.h"

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
}

VisionPose DragonPhotonCam::GetFieldPosition()
{ /*
    visionapi - add traditional way

     double poseAmbiguity = target.GetPoseAmbiguity();
     frc::Transform3d bestCameraToTarget = target.getBestCameraToTarget();*/
    // above maps camera space to object space, transform camera position to get to apriltag

    /*
        Pose Ambiguity plus possibly robot yaw compared to actual (pigeon, method from MechanicalAdvantage)
        can be a measure of confidence in the pose, possibly manipulate standard deviations

        will need PhotonPoseEstimator

        PoseEstimator should probably be located in vision drive
        This should return "traditional" way to calculate field pose by transforming from known apriltag field position
    */

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

        // std::optional<frc::Pose3d> potentialPose = frc::AprilTagFieldLayout::GetTagPose(tagId);

        /*
        if(potentialPose.has_value())
        {
            fieldRelativeTagPose.plus(cameraToTarget.inverse()).plus(cameraToRobot);

            //initial pose = Tag (field relative)
            //transform target -> cam (reason for inverse is we get cam -> target, direction matters)
            //then transform cam -> robot (again, need to inverse since we get robot -> cam)
            frc::Pose3d fieldRelPose = potentialPose.value().plus(camToTargetTransform.inverse()).plus(m_robotToCamTransform.inverse());

            timestamp = frc::FPGAGetTimeStamp();

            visionStdMeasurements = default(0.1) / target.GetPoseAmbiguity();

            return VisionPose(fieldRelPose, timestamp, visionStdMeasurements);
        }

        */
    }
}

VisionPose DragonPhotonCam::GetFieldPosition(frc::DriverStation::Alliance alliance)
{
    return GetFieldPosition();
}

/// @brief  get Yaw of possible target.
/// @return units::angle::degree_t - Counter Clockwize/left for positive.
units::angle::degree_t DragonPhotonCam::GetTargetYaw() const // originally Yaw was y-angle
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
    return (units::angle::degree_t)0;
}

/// @brief get TargetSkew of possible target.
/// @return Skew casted as units::angle::degree_t. Counter Clockwize/left for positive.
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
    return (units::angle::degree_t)0;
}

units::angle::degree_t DragonPhotonCam::GetTargetYawRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const {}

units::angle::degree_t DragonPhotonCam::GetTargetPitchRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const {}

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
    return (units::angle::degree_t)0;
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

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();
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

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();
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

        // Get transformation from camera to target
        frc::Transform3d transform = target.GetBestCameraToTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().Z() + m_robotCenterToCam.Z();
    }

    return units::length::inch_t(-1.0);
}
bool DragonPhotonCam::SetPipeline(DragonCamera::PIPELINE pipeline)
{
    return false;
}