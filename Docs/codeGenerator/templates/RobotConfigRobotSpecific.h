$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include "configs/RobotConfig.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/base/StateMgr.h"
$$_MECHANISM_INCLUDE_FILES_$$

class RobotConfig$$_ROBOT_NAME_$$ : public RobotConfig
{
public:
    RobotConfig$$_ROBOT_NAME_$$() = default;
    ~RobotConfig$$_ROBOT_NAME_$$() = default;

    StateMgr *GetMechanism(MechanismTypes::MECHANISM_TYPE mechType);

protected:
    void DefineMechanisms() override;
    void DefineVisionSensors() override;
    void DefineLEDs() override;

private:
    $$_MECHANISM_PTR_DECLARATIONS_$$

    std::unordered_map<MechanismTypes::MECHANISM_TYPE, StateMgr *> m_mechanismMap;

    $$_CAMERA_PTR_DECLARATIONS_$$
};