$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

#include "configs/RobotConfig.h"

class RobotConfigMgr
{
public:
    enum RobotIdentifier
    {
        $$_ROBOT_CONFIGURATIONS_NAMES_ENUMS_$$

        MAX_ROBOT_IDENTIFIERS
    };

    static RobotConfigMgr* GetInstance();
    RobotConfig* GetCurrentConfig() const { return m_config; }
    void InitRobot(RobotIdentifier);

private:
    RobotConfigMgr();
    ~RobotConfigMgr() = default;

    static RobotConfigMgr* m_instance;
    RobotConfig* m_config;
};
