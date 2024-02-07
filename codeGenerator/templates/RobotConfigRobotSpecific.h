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
    void DefineVisionSensors() override;

private:
    $$_MECHANISM_PTR_DECLARATIONS_$$
    $$_CAMERA_PTR_DECLARATIONS_$$
};