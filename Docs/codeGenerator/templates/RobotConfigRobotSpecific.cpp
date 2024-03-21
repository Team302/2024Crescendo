$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig$$_ROBOT_NAME_$$.h"
#include "configs/RobotElementNames.h"
$$_INCLUDE_$$

using std::string;

void RobotConfig$$_ROBOT_NAME_$$::DefineMechanisms()
{
    $$_MECHANISMS_INITIALIZATION_$$
}

void RobotConfig$$_ROBOT_NAME_$$::DefineLEDs(){
    $$_LED_INITIALIZATION_$$

}

StateMgr *RobotConfig$$_ROBOT_NAME_$$::GetMechanism(MechanismTypes::MECHANISM_TYPE mechType)
{
    auto itr = m_mechanismMap.find(mechType);
    if (itr != m_mechanismMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

void RobotConfig$$_ROBOT_NAME_$$::DefineVisionSensors()
{
    $$_CAMERAS_INITIALIZATION_$$
}
