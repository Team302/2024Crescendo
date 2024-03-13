#include "DragonVision/DragonPhotonCalculator.h"

units::length::meter_t DragonPhotonCalculator::GetElementVerticalOffset()
{
    return units::length::inch_t(0.0);
}

std::optional<units::length::inch_t> DragonPhotonCalculator::EstimateTargetXDistance_RelToRobotCoords(frc::Transform3d robotCenterToCamTransform, photon::PhotonPipelineResult result)
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().X() + robotCenterToCamTransform.X();
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonPhotonCalculator::EstimateTargetYDistance_RelToRobotCoords(frc::Transform3d robotCenterToCamTransform, photon::PhotonPipelineResult result)
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through
    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().Y() + robotCenterToCamTransform.Y();
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonPhotonCalculator::EstimateTargetZDistance_RelToRobotCoords(frc::Transform3d robotCenterToCamTransform, photon::PhotonPipelineResult result)
{
    ///@TODO: May have problems when Multi-tag is enabled, data may not come through

    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate data according to contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        // just need to add translation components of transforms together (camToTarget.X() + robotToCam.X())
        return target.GetBestCameraToTarget().Z() + robotCenterToCamTransform.Z();
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCalculator::GetTargetPitchRobotFrame(frc::Pose3d cameraPose, photon::PhotonPipelineResult result)
{
    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        return GetTargetPitchRobotFrame(cameraPose, target);
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCalculator::GetTargetPitchRobotFrame(frc::Pose3d cameraPose, photon::PhotonTrackedTarget target)
{
    int tagId = target.GetFiducialId();

    // if we don't see a tag we are detecting a note instead
    if (tagId == -1)
    {
        // Use photon utils to calculate distance to target
        units::meter_t distanceToTarget = photon::PhotonUtils::CalculateDistanceToTarget(cameraPose.Z(),                      // camera mounting height
                                                                                         GetElementVerticalOffset(),          // offset from robot center based on note being on the floor
                                                                                         cameraPose.Rotation().Y(),           // camera mounting pitch
                                                                                         units::degree_t{target.GetPitch()}); // pitch of detection

        // inverse tangent of opposite (z/height) over adjacent (x distance)
        units::angle::radian_t pitchRobotRelative = units::math::atan2(cameraPose.Z(), distanceToTarget);

        return pitchRobotRelative;
    }
    else // we see an april tag
    {
        // transform to get from cam to target
        frc::Transform3d camToTarget = target.GetBestCameraToTarget();

        // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
        units::angle::radian_t pitchRobotRelative = units::math::atan2(frc::Transform3d(frc::Pose3d{}, (cameraPose + camToTarget)).Z(), frc::Transform3d(frc::Pose3d{}, (cameraPose + camToTarget)).X());

        return pitchRobotRelative;
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCalculator::GetTargetYawRobotFrame(frc::Pose3d cameraPose, photon::PhotonPipelineResult result)
{
    // check for detections
    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        return GetTargetYawRobotFrame(cameraPose, target);
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonPhotonCalculator::GetTargetYawRobotFrame(frc::Pose3d cameraPose, photon::PhotonTrackedTarget target)
{
    int tagId = target.GetFiducialId();

    // if we don't see a tag we are detecting a note instead
    if (tagId == -1)
    {
        // Use photon utils to calculate distance to target
        units::meter_t distanceToTarget = photon::PhotonUtils::CalculateDistanceToTarget(cameraPose.Z(),                      // camera mounting height
                                                                                         GetElementVerticalOffset(),          // offset from robot center based on note being on the floor
                                                                                         cameraPose.Rotation().Y(),           // camera mounting pitch
                                                                                         units::degree_t{target.GetPitch()}); // pitch of detection

        // get y distance to camera by multiplying distanceToTarget(x distance/adjacent) to the tangent of the yaw (opposite = ydistance, adjacent = x distance)
        units::length::meter_t yOffsetCamToTarget = distanceToTarget * units::math::tan(units::degree_t{target.GetYaw()});

        // Add camera y offset from robot center to target's y offset from cam
        units::length::meter_t yOffsetRobotToTarget = yOffsetCamToTarget + cameraPose.Y();

        // inverse tangent of opposite (y/ left/right offset) over adjacent (x distance)
        units::angle::radian_t yawRobotRelative = units::math::atan2(yOffsetRobotToTarget, distanceToTarget);

        return yawRobotRelative;
    }
    else // we see an april tag
    {
        // transform to get from cam to target
        frc::Transform3d camToTarget = target.GetBestCameraToTarget();

        // inverse tangent of opposite (sum of camera mounting height and camera to target) over adjacent (sum of camera mounting x offset and cam to target x distance)
        units::angle::radian_t yawRobotRelative = units::math::atan2(camToTarget.Y() - cameraPose.Y(), camToTarget.X() - cameraPose.X());

        return yawRobotRelative;
    }

    return std::nullopt;
}

std::optional<VisionData> DragonPhotonCalculator::GetDataToNearestAprilTag(frc::Pose3d cameraPose, photon::PhotonPipelineResult result)
{

    if (result.HasTargets())
    {
        // get the most accurate according to configured contour ranking
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        frc::Transform3d camToTargetTransform = target.GetBestCameraToTarget();

        frc::Transform3d robotToTargetTransform = frc::Transform3d{frc::Pose3d{}, (cameraPose + camToTargetTransform)};

        std::optional<units::angle::degree_t> pitch = GetTargetPitchRobotFrame(cameraPose, result);
        std::optional<units::angle::degree_t> yaw = GetTargetYawRobotFrame(cameraPose, result);

        if (pitch && yaw)
        {
            frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(0.0), // roll
                                                       pitch.value(),               // pitch
                                                       yaw.value()};                // yaw

            return VisionData{robotToTargetTransform, robotToTargetTransform.Translation(), rotation, target.GetFiducialId()};
        }
    }

    return std::nullopt;
}

std::optional<VisionData> DragonPhotonCalculator::GetDataToNearestAprilTag(frc::Pose3d cameraPose, photon::PhotonTrackedTarget target)
{
    frc::Transform3d camToTargetTransform = target.GetBestCameraToTarget();

    frc::Transform3d robotToTargetTransform = frc::Transform3d{frc::Pose3d{}, (cameraPose + camToTargetTransform)};

    std::optional<units::angle::degree_t> pitch = GetTargetPitchRobotFrame(cameraPose, target);
    std::optional<units::angle::degree_t> yaw = GetTargetYawRobotFrame(cameraPose, target);

    if (pitch && yaw)
    {
        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(0.0), // roll
                                                   pitch.value(),               // pitch
                                                   yaw.value()};                // yaw

        return VisionData{robotToTargetTransform, robotToTargetTransform.Translation(), rotation, target.GetFiducialId()};
    }

    return std::nullopt;
}
