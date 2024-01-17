$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_gen.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/decoratormods/$$_MECHANISM_INSTANCE_NAME_$$.h"

using std::string;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
$$_MECHANISM_INSTANCE_NAME_$$::$$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$_gen *base) : $$_MECHANISM_INSTANCE_NAME_$$_gen(),
                                                                                                        m_$$_MECHANISM_INSTANCE_NAME_$$(base)
{
    // PeriodicLooper::GetInstance()->RegisterAll(*this);
}

// todo not sure what to do with this
/*
bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
    return m_$$_MECHANISM_INSTANCE_NAME_$$->IsAtMinPosition(identifier);
}
bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
    return m_$$_MECHANISM_INSTANCE_NAME_$$->IsAtMinPosition(identifier);
}
bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
    return m_$$_MECHANISM_INSTANCE_NAME_$$->IsAtMaxPosition(identifier);
}
bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
    return m_$$_MECHANISM_INSTANCE_NAME_$$->IsAtMaxPosition(identifier);
}
*/
