$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

// C++ Includes
#include <string>

// FRC Includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$Gen.h"
#include "mechanisms/base/StateMgr.h"

// forward declares

class $$_MECHANISM_INSTANCE_NAME_$$ : public $$_MECHANISM_INSTANCE_NAME_$$Gen
{
public:
    /// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
    /// @param controlFileName The control file with the PID constants and Targets for each state
    /// @param networkTableName Location for logging information
    /// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
    /// @param otherMotor Same as previous
    /// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
    /// Additional actuators and sensors are also in this list.
    $$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$Gen *generatedMech, RobotConfigMgr::RobotIdentifier activeRobotId);
    $$_MECHANISM_INSTANCE_NAME_$$() = delete;
    ~$$_MECHANISM_INSTANCE_NAME_$$() = default;

    void RunCommonTasks() override;
    void SetCurrentState(int state, bool run) override;
    void CreateAndRegisterStates();

    RobotConfigMgr::RobotIdentifier getActiveRobotId() { return m_activeRobotId; }

private:
    $$_MECHANISM_INSTANCE_NAME_$$Gen *m_$$_MECHANISM_INSTANCE_NAME_$$;
};
