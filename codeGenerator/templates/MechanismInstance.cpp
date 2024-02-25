$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/decoratormods/$$_MECHANISM_INSTANCE_NAME_$$.h"

$$_STATE_CLASSES_INCLUDES_$$

using std::string;
using namespace $$_MECHANISM_INSTANCE_NAME_$$States;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
$$_MECHANISM_INSTANCE_NAME_$$::$$_MECHANISM_INSTANCE_NAME_$$($$_MECHANISM_INSTANCE_NAME_$$Gen *base, RobotConfigMgr::RobotIdentifier activeRobotId) : $$_MECHANISM_INSTANCE_NAME_$$Gen(activeRobotId),
                                                                                                                                                      m_$$_MECHANISM_INSTANCE_NAME_$$(base)
{
    PeriodicLooper::GetInstance()->RegisterAll(this);
}

void $$_MECHANISM_INSTANCE_NAME_$$::RunCommonTasks()
{
    // This function is called once per loop before the current state Run()
}

void $$_MECHANISM_INSTANCE_NAME_$$::SetCurrentState(int state, bool run)
{
    $$_MECHANISM_INSTANCE_NAME_$$Gen::SetCurrentState(state, run);
}