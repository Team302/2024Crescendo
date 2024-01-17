$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
$$_ROBOT_CONFIG_INCLUDES_$$

using namespace std;

RobotConfigMgr *RobotConfigMgr::m_instance = nullptr;
RobotConfigMgr *RobotConfigMgr::GetInstance()
{
    if (RobotConfigMgr::m_instance == nullptr)
    {
        RobotConfigMgr::m_instance = new RobotConfigMgr();
    }
    return RobotConfigMgr::m_instance;
}

RobotConfigMgr::RobotConfigMgr() : m_config(nullptr)
{
}

void RobotConfigMgr::InitRobot(RobotIdentifier id)
{
    switch (id)
    {
        $$_ROBOT_CONFIGURATION_CREATION_$$

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Skipping robot initialization because of unknown robot id "), string(""), id);
        break;
    }

    if (m_config != nullptr)
    {
        m_config->BuildRobot();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Initialization completed for robot "), string(""), id);
    }
}
