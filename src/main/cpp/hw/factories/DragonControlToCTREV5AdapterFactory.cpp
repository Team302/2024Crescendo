
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

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "hw/DistanceAngleCalcStruc.h"
#include "hw/ctreadapters/v5/DragonControlToCTREV5Adapter.h"
#include "hw/factories/DragonControlToCTREV5AdapterFactory.h"
#include "mechanisms/controllers/ControlData.h"
#include "utils/logging/Logger.h"

#include "hw/ctreadapters/v5/DragonControlToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonPercentOutputToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonPositionDegreeToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonPositionInchToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonTicksToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonTrapezoidToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonVelocityDegreeToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonVelocityInchToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonVelocityRPSToCTREV5Adapter.h"
#include "hw/ctreadapters/v5/DragonVoltageToCTREV5Adapter.h"

// Third Party Includes
#include "wpi/deprecated.h"
WPI_IGNORE_DEPRECATED
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
WPI_UNIGNORE_DEPRECATED

using namespace std;

DragonControlToCTREV5AdapterFactory *DragonControlToCTREV5AdapterFactory::m_factory = nullptr;

//=======================================================================================
/// Method: GetInstance
/// @brief  Get the factory singleton
/// @return DragonServoFactory*    pointer to the factory
//=======================================================================================
DragonControlToCTREV5AdapterFactory *DragonControlToCTREV5AdapterFactory::GetFactory()
{
    if (DragonControlToCTREV5AdapterFactory::m_factory == nullptr)
    {
        DragonControlToCTREV5AdapterFactory::m_factory = new DragonControlToCTREV5AdapterFactory();
    }
    return DragonControlToCTREV5AdapterFactory::m_factory;
}

DragonControlToCTREV5Adapter *DragonControlToCTREV5AdapterFactory::CreatePercentOuptutAdapter(std::string networkTableName,
                                                                                              ctre::phoenix::motorcontrol::can::TalonSRX *controller)
{
    ControlData controlInfo;
    DistanceAngleCalcStruc calcStruc;
    return CreateAdapter(networkTableName, 0, controlInfo, calcStruc, controller);
}
DragonControlToCTREV5Adapter *DragonControlToCTREV5AdapterFactory::CreateAdapter(std::string networkTableName,
                                                                                 int controllerSlot,
                                                                                 const ControlData &controlInfo,
                                                                                 const DistanceAngleCalcStruc &calcStruc,
                                                                                 ctre::phoenix::motorcontrol::can::TalonSRX *controller)
{
    if (controller != nullptr)
    {
        switch (controlInfo.GetMode())
        {
        case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
            return new DragonPercentOutputToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::POSITION_ABS_TICKS:
            return new DragonTicksToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
            return new DragonPositionDegreeToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::POSITION_INCH:
            return new DragonPositionInchToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
            return new DragonPercentOutputToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::TRAPEZOID:
            return new DragonTrapezoidToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
            return new DragonVelocityDegreeToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
            return new DragonVelocityInchToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
            return new DragonVelocityRPSToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        case ControlModes::CONTROL_TYPE::VOLTAGE:
            return new DragonVoltageToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;

        default:
            string msg{"Invalid control data "};
            msg += to_string(controller->GetDeviceID());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonControlToCTREV5AdapterFactory"), string("CreateAdapter"), msg);
            return new DragonPercentOutputToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
            break;
        }
    }
    string msg{"Invalid contrrol information "};
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonControlToCTREV5AdapterFactory"), string("CreateAdapter"), msg);
    return new DragonPercentOutputToCTREV5Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller);
}
