#pragma once

#include <string>
#include "photon/targeting/PhotonPipelineResult.h"
#include "photon/PhotonUtils.h"
#include <DragonVision/DragonVisionStructs.h>

class DragonPhotonCalculator
{
public:
    static units::length::meter_t GetElementVerticalOffset();
    static std::optional<units::angle::degree_t> GetTargetPitchRobotFrame(frc::Pose3d cameraPose, photon::PhotonPipelineResult result);
    static std::optional<units::angle::degree_t> GetTargetYawRobotFrame(frc::Pose3d cameraPose, photon::PhotonPipelineResult result);
    static std::optional<VisionData> GetDataToNearestAprilTag(frc::Pose3d cameraPose, photon::PhotonPipelineResult result);
    
};

