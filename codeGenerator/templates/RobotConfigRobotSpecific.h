$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include "configs/RobotConfig.h"
$$_MECHANISM_INCLUDE_FILES_$$

class RobotConfig$$_ROBOT_NAME_$$ : public RobotConfig
{
public:
    RobotConfig$$_ROBOT_NAME_$$() = default;
    ~RobotConfig$$_ROBOT_NAME_$$() = default;

protected:
    void DefineMechanisms() override;

private:
    $$_MECHANISM_PTR_DECLARATIONS_$$
};