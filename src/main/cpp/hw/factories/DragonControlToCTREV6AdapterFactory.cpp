
//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes

// Team 302 includes
#include "hw/factories/DragonControlToCTREV6AdapterFactory.h"
#include "hw/ctreadapters/v6/DragonControlToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonPercentOutputToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonPositionDegreeToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonPositionInchToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonTicksToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonTrapezoidToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonVelocityDegreeToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonVelocityInchToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonVelocityRPSToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonVelocityToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonVoltageToCTREV6Adapter.h"
#include "utils/logging/Logger.h"

using namespace std;

DragonControlToCTREV6AdapterFactory *DragonControlToCTREV6AdapterFactory::m_factory = nullptr;

//=======================================================================================
/// Method: GetInstance
/// @brief  Get the factory singleton
/// @return DragonServoFactory*    pointer to the factory
//=======================================================================================
DragonControlToCTREV6AdapterFactory *DragonControlToCTREV6AdapterFactory::GetFactory()
{
    if (DragonControlToCTREV6AdapterFactory::m_factory == nullptr)
    {
        DragonControlToCTREV6AdapterFactory::m_factory = new DragonControlToCTREV6AdapterFactory();
    }
    return DragonControlToCTREV6AdapterFactory::m_factory;
}

DragonControlToCTREV6Adapter *DragonControlToCTREV6AdapterFactory::CreateAdapter(std::string networkTableName,
                                                                                 int controllerSlot,
                                                                                 const ControlData &controlInfo,
                                                                                 const DistanceAngleCalcStruc &calcStruc,
                                                                                 ctre::phoenix6::hardware::TalonFX &controller)
{
    switch (controlInfo.GetMode())
    {
    case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
        return new DragonPercentOutputToCTREV6Adapter(networkTableName,
                                                      controllerSlot,
                                                      controlInfo,
                                                      calcStruc,
                                                      controller);
        break;
    case ControlModes::CONTROL_TYPE::POSITION_ABS_TICKS:
        return new DragonTicksToCTREV6Adapter(networkTableName,
                                              controllerSlot,
                                              controlInfo,
                                              calcStruc,
                                              controller);
        break;

    case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
        return new DragonPositionDegreeToCTREV6Adapter(networkTableName,
                                                       controllerSlot,
                                                       controlInfo,
                                                       calcStruc,
                                                       controller);
        break;

    case ControlModes::CONTROL_TYPE::POSITION_INCH:
        return new DragonPositionInchToCTREV6Adapter(networkTableName,
                                                     controllerSlot,
                                                     controlInfo,
                                                     calcStruc,
                                                     controller);
        break;

    case ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE:
        return new DragonPositionDegreeToCTREV6Adapter(networkTableName,
                                                       controllerSlot,
                                                       controlInfo,
                                                       calcStruc,
                                                       controller);
        break;

    case ControlModes::CONTROL_TYPE::TRAPEZOID:
        return new DragonTrapezoidToCTREV6Adapter(networkTableName,
                                                  controllerSlot,
                                                  controlInfo,
                                                  calcStruc,
                                                  controller);
        break;

    case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
        return new DragonVelocityDegreeToCTREV6Adapter(networkTableName,
                                                       controllerSlot,
                                                       controlInfo,
                                                       calcStruc,
                                                       controller);
        break;

    case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
        return new DragonVelocityInchToCTREV6Adapter(networkTableName,
                                                     controllerSlot,
                                                     controlInfo,
                                                     calcStruc,
                                                     controller);
        break;

    case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
        return new DragonVelocityRPSToCTREV6Adapter(networkTableName,
                                                    controllerSlot,
                                                    controlInfo,
                                                    calcStruc,
                                                    controller);
        break;

    case ControlModes::CONTROL_TYPE::VOLTAGE:
        return new DragonVoltageToCTREV6Adapter(networkTableName,
                                                controllerSlot,
                                                controlInfo,
                                                calcStruc,
                                                controller);
        break;

    default:
        string msg{"Invalid control data "};
        msg += to_string(controller.GetDeviceID());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonControlToCTREV6AdapterFactory"), string("CreateAdapter"), msg);
        return new DragonPercentOutputToCTREV6Adapter(networkTableName,
                                                      controllerSlot,
                                                      controlInfo,
                                                      calcStruc,
                                                      controller);
        break;
    }
    return nullptr;
}
