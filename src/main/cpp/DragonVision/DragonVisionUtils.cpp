#include "DragonVision/DragonVisionUtils.h"
#include "utils/logging/Logger.h"

bool DragonVisionUtils::CompareVisionData(std::optional<VisionData> data1, std::optional<VisionData> data2)
{
    if (data1.has_value() && data2.has_value())
    {
        frc::Transform3d transform1 = data1.value().transformToTarget;
        frc::Transform3d transform2 = data2.value().transformToTarget;
        if (transform1.X() == transform2.X() && transform1.Y() == transform2.Y() && transform1.Z() == transform2.Z())
        {

            if (transform1.Rotation().Angle() == transform2.Rotation().Angle() && transform1.Rotation().X() == transform2.Rotation().X() && transform1.Rotation().Y() == transform2.Rotation().Y())
            {

                frc::Translation3d translation1 = data1.value().translationToTarget;
                frc::Translation3d translation2 = data2.value().translationToTarget;
                if (translation1.X() == translation2.X() && translation1.Y() == translation2.Y() && translation1.Z() == translation2.Z())
                {

                    frc::Rotation3d rotation1 = data1.value().rotationToTarget;
                    frc::Rotation3d rotation2 = data2.value().rotationToTarget;
                    if (rotation1.Angle() == rotation2.Angle() && rotation1.X() == rotation2.X() && rotation1.Y() == rotation2.Y())
                    {

                        if (data1.value().tagId == data2.value().tagId)
                        {

                            return true;
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("tagId"), std::string("NOT EQUAL!"));
                        }
                    }
                    else
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("rotationToTarget"), std::string("NOT EQUAL!"));
                    }
                }
                else
                {
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("translationToTarget"), std::string("NOT EQUAL!"));
                }
            }
            else
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("transformToTarget"), std::string("NOT EQUAL!"));
            }
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("transformToTarget"), std::string("NOT EQUAL!"));
        }
    }
    else if (!data1.has_value() && !data2.has_value())
    {
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("CompareData"), std::string("NULL"), std::string("BOTH!"));
        return true;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("CompareData"), std::string("data1 or data2 compare"), std::string("NOT EQUAL!"));
    }

    return false;
}