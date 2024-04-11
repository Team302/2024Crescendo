#pragma once

#include <string>
#include "DragonVision/DragonVisionStructs.h"
#include "DragonVision/DragonCamera.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Transform3d.h"

class DragonVisionStructLogger
{
public:
    static void logVisionData(const std::string& loggerName, const std::optional<VisionData> optVisionData);
    static void logTransform3d(const std::string &loggerName, const frc::Transform3d transform3d);
    static void logTranslation3d(const std::string &loggerName, const frc::Translation3d translation3d);
    static void logRotation3d(const std::string &loggerName, const frc::Rotation3d rotation3d);
    static void logDragonCamera(const std::string &loggerName, const DragonCamera& camera);
    static void logPose3d(const std::string &loggerName, const frc::Pose3d pose3d);
    static void logVisionPose(const std::string &loggerName, const std::optional<VisionPose> optVisionPose);
    static void logPose2d(const std::string &loggerName, const frc::Pose2d pose2d);
    
};