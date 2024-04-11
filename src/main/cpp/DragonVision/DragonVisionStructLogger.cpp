#include "DragonVision/DragonVisionStructLogger.h"
#include "utils/logging/Logger.h"

/************
 * Function: logVisionData
 * Description: Logs the vision data to the logger
 * Parameters: const std::string& loggerName, const std::optional<VisionData> optVisionData
 * Returns: void
 */
void DragonVisionStructLogger::logVisionData(const std::string &loggerName, const std::optional<VisionData> optVisionData)
{
    if (optVisionData)
    {
        logTransform3d(loggerName, optVisionData.value().transformToTarget);
        logTranslation3d(loggerName, optVisionData.value().translationToTarget);
        logRotation3d(loggerName, optVisionData.value().rotationToTarget);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, loggerName, std::string("X"), std::string("No vision data found"));
    }
}

/*******************
 * Function: logTransform3d
 * Description: Logs the transform3d to the logger
 * Parameters: const std::string& loggerName, const frc::Transform3d transform3d
 * Returns: void
 *
 */
void DragonVisionStructLogger::logTransform3d(const std::string &loggerName, const frc::Transform3d transform3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("X"), std::to_string(transform3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Y"), std::to_string(transform3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Z"), std::to_string(transform3d.Z().to<double>()));
}

/*******************
 * Function: logTranslation3d
 * Description: Logs the translation3d to the logger
 * Parameters: const std::string& loggerName, const frc::Translation3d translation3d
 * Returns: void
 *
 */
void DragonVisionStructLogger::logTranslation3d(const std::string &loggerName, const frc::Translation3d translation3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationX"), std::to_string(translation3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationY"), std::to_string(translation3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationZ"), std::to_string(translation3d.Z().to<double>()));
}

/*******************
 * Function: logRotation3d
 * Description: Logs the rotation3d to the logger
 * Parameters: const std::string& loggerName, const frc::Rotation3d rotation3d
 * Returns: void
 *
 ***/
void DragonVisionStructLogger::logRotation3d(const std::string &loggerName, const frc::Rotation3d rotation3d)
{
    units::angle::degree_t roll = rotation3d.X();
    units::angle::degree_t pitch = rotation3d.Y();
    units::angle::degree_t yaw = rotation3d.Z();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("x:Roll"), std::to_string(roll.to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("y:Pitch"), std::to_string(pitch.to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("z:Yaw"), std::to_string(yaw.to<double>()));
}

/**************
 * Function: logDragonCamera
 * Description: Logs the dragon camera to the logger
 * Parameters: const std::string& loggerName, const DragonCamera& camera
 * Returns: void
 *
 */
void DragonVisionStructLogger::logDragonCamera(const std::string &loggerName, const DragonCamera &camera)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraName"), camera.GetCameraName());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Pipeline"), std::to_string(camera.GetPipeline()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingXOffset"), std::to_string(camera.GetMountingXOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingYOffset"), std::to_string(camera.GetMountingYOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingZOffset"), std::to_string(camera.GetMountingZOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraPitch"), std::to_string(camera.GetCameraPitch().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraYaw"), std::to_string(camera.GetCameraYaw().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraRoll"), std::to_string(camera.GetCameraRoll().to<double>()));
}

void DragonVisionStructLogger::logPose3d(const std::string &loggerName, const frc::Pose3d pose3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("X"), std::to_string(pose3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Y"), std::to_string(pose3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Z"), std::to_string(pose3d.Z().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Roll"), std::to_string(pose3d.Rotation().X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Pitch"), std::to_string(pose3d.Rotation().Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Yaw"), std::to_string(pose3d.Rotation().Z().to<double>()));
}

/****************
 * Function: logVisionPose
 * Description: Logs the vision pose to the logger
 * Parameters: const std::string& loggerName, const std::optional<VisionPose>& optVisionPose
 * Returns: void
 *
 */
void DragonVisionStructLogger::logVisionPose(const std::string &loggerName, const std::optional<VisionPose> optVisionPose)
{
    if (optVisionPose)
    {
        frc::Pose3d pose = optVisionPose.value().estimatedPose;
        logPose3d(loggerName, pose);

        units::time::millisecond_t timeStamp = optVisionPose.value().timeStamp;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("TimeStamp"), std::to_string(timeStamp.to<double>()));

        wpi::array<double, 3> stdDevs = optVisionPose.value().visionMeasurementStdDevs;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("StdDevX"), std::to_string(stdDevs[0]));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("StdDevY"), std::to_string(stdDevs[1]));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("StdDevZ"), std::to_string(stdDevs[2]));
        PoseEstimationStrategy strategy = optVisionPose.value().estimationStrategy;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("PoseEstimationStrategy"), std::to_string(strategy));
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, loggerName, std::string("X"), std::string("No vision pose found"));
    }
}

void DragonVisionStructLogger::logPose2d(const std::string &loggerName, const frc::Pose2d pose2d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("X"), std::to_string(pose2d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Y"), std::to_string(pose2d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Rotation"), std::to_string(pose2d.Rotation().Degrees().to<double>()));
}