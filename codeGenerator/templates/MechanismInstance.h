$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

// C++ Includes

// FRC Includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_gen.h"
#include "mechanisms/base/StateMgr.h"

// forward declares

class $$_MECHANISM_INSTANCE_NAME_$$ : public $$_MECHANISM_INSTANCE_NAME_$$_gen
{
public:
    /// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
    /// @param controlFileName The control file with the PID constants and Targets for each state
    /// @param networkTableName Location for logging information
    /// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
    /// @param otherMotor Same as previous
    /// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
    /// Additional actuators and sensors are also in this list.
    $$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$_gen* generatedMech);
    $$_MECHANISM_INSTANCE_NAME_$$() = delete;
    ~$$_MECHANISM_INSTANCE_NAME_$$() = default;

    //todo not sure what to do with these
    /*
    bool IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
    bool IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
    bool IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
    bool IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
    */

private:
    $$_MECHANISM_INSTANCE_NAME_$$_gen* m_$$_MECHANISM_INSTANCE_NAME_$$;
};
