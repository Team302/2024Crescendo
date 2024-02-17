#include "DragonVision/DragonVisionStructLogger.h"
#include "utils/logging/Logger.h"

void DragonVisionStructLogger::logVisionData( const std::string& loggerName, const std::optional<VisionData> optVisionData)
{
    if (optVisionData)
    {
        frc::Transform3d testTransform = optVisionData.value().deltaToTarget;
        logTransform3d(loggerName, testTransform);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("ERROR"), std::string("No vision data found"));
    }

}

void DragonVisionStructLogger::logTransform3d(const std::string &loggerName, const frc::Transform3d transform3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("X"), std::to_string(transform3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Y"), std::to_string(transform3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Z"), std::to_string(transform3d.Z().to<double>()));

}

void DragonVisionStructLogger::logTranslation3d(const std::string &loggerName, const frc::Translation3d translation3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationX"), std::to_string(translation3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationY"), std::to_string(translation3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("translationZ"), std::to_string(translation3d.Z().to<double>()));
}

void DragonVisionStructLogger::logRotation3d(const std::string &loggerName, const frc::Rotation3d rotation3d)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("x:Roll"), std::to_string(rotation3d.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("y:Pitch"), std::to_string(rotation3d.Y().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("z:Yaw"), std::to_string(rotation3d.Z().to<double>()));
}

void DragonVisionStructLogger::logDragonCamera(const std::string &loggerName, const DragonCamera &camera){
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraName"), camera.GetCameraName());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("Pipeline"), std::to_string(camera.GetPipeline()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingXOffset"), std::to_string(camera.GetMountingXOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingYOffset"), std::to_string(camera.GetMountingYOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("MountingZOffset"), std::to_string(camera.GetMountingZOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraPitch"), std::to_string(camera.GetCameraPitch().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraYaw"), std::to_string(camera.GetCameraYaw().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, loggerName, std::string("CameraRoll"), std::to_string(camera.GetCameraRoll().to<double>()));
}