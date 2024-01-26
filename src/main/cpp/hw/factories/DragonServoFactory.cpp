
//====================================================================================================================================================
// Copyright 2024 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <map>
#include <memory>
#include <string>

#include "hw/factories/DragonServoFactory.h"
#include "hw/DragonServo.h"
#include "configs/RobotElementNames.h"

#include "utils/logging/Logger.h"

using namespace std;

DragonServoFactory *DragonServoFactory::m_instance = nullptr;

//=======================================================================================
/// Method: GetInstance
/// @brief  Get the factory singleton
/// @return DragonServoFactory*    pointer to the factory
//=======================================================================================
DragonServoFactory *DragonServoFactory::GetInstance()
{
    if (DragonServoFactory::m_instance == nullptr)
    {
        DragonServoFactory::m_instance = new DragonServoFactory();
    }
    return DragonServoFactory::m_instance;
}

//=======================================================================================
/// Method: CreateDragonServo
/// @brief  Create a DragonServo from the inputs
/// @param [in] DragonServo::SERVO_USAGE   deviceUsage  Usage of the servo
/// @param [in] int                        deviceID     PWM ID of the  servo
/// @param [in] double                     minAngle     Minimum Angle for the servo
/// @param [in] double                     maxAngle     Maximum Angle for the servo
/// @return std::shared_ptr<DragonServo>    - could be nullptr if invalid inputs are supplied
//=======================================================================================
DragonServo *DragonServoFactory::CreateDragonServo(
    string networkTableName,
    RobotElementNames::SERVO_USAGE deviceUsage,
    int deviceID,
    units::angle::degree_t minAngle,
    units::angle::degree_t maxAngle)
{
    if (deviceUsage > RobotElementNames::SERVO_USAGE::UNKNOWN_SERVO &&
        deviceUsage < RobotElementNames::SERVO_USAGE::MAX_SERVO)
    {
        m_servos[deviceUsage] = new DragonServo(deviceUsage, deviceID, minAngle, maxAngle);
        return m_servos[deviceUsage];
    }

    string msg = "Unknown Servo Usage " + to_string(deviceUsage);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonServoFactory"), string("CreateDragonServo"), msg);
    return nullptr;
}

//=======================================================================================
/// Method: GetDragonServo
/// @brief  Get a DragonServo from its usage
/// @param [in] DragonServo::SERVO_USAGE   deviceUsage  Usage of the servo
/// @return std::shared_ptr<DragonServo>    - could be nullptr if invalid inputs are supplied
//=======================================================================================
DragonServo *DragonServoFactory::GetDragonServo(
    RobotElementNames::SERVO_USAGE deviceUsage)
{
    return m_servos.find(deviceUsage)->second;
}
