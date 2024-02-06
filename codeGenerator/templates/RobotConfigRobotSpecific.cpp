$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig$$_ROBOT_NAME_$$.h"

using std::string;

void RobotConfig$$_ROBOT_NAME_$$::DefineMechanisms(){
    $$_MECHANISMS_INITIALIZATION_$$

}

StateMgr *RobotConfig$$_ROBOT_NAME_$$::getMechanism(MechanismTypes::MECHANISM_TYPE mechType)
{
    auto itr = m_mechanismMap.find(mechType);
    if (itr != m_mechanismMap.end())
    {
        return itr->second;
    }
    return nullptr;
}
