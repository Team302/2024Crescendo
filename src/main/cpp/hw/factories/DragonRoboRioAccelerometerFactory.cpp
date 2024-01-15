
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

// FRC includes

// Team 302 includes
#include "hw/builtinaccel/DragonBuiltInAccelerometer.h"
#include "hw/builtinaccel/DragonRoboRioAccelXBackwardYLeft.h"
#include "hw/builtinaccel/DragonRoboRioAccelXBackwardYRight.h"
#include "hw/builtinaccel/DragonRoboRioAccelXDownYRight.h"
#include "hw/builtinaccel/DragonRoboRioAccelXDownYLeft.h"
#include "hw/builtinaccel/DragonRoboRioAccelXLeftYBackward.h"
#include "hw/builtinaccel/DragonRoboRioAccelXLeftYDown.h"
#include "hw/builtinaccel/DragonRoboRioAccelXLeftYForward.h"
#include "hw/builtinaccel/DragonRoboRioAccelXLeftYUp.h"
#include "hw/builtinaccel/DragonRoboRioAccelXRightYBackward.h"
#include "hw/builtinaccel/DragonRoboRioAccelXRightYDown.h"
#include "hw/builtinaccel/DragonRoboRioAccelXRightYForward.h"
#include "hw/builtinaccel/DragonRoboRioAccelXRightYUp.h"
#include "hw/builtinaccel/DragonRoboRioAccelXUpYLeft.h"
#include "hw/builtinaccel/DragonRoboRioAccelXUpYRight.h"
#include "hw/builtinaccel/DragonRoboRioAccelXForwardYRight.h"
#include "hw/factories/DragonRoboRioAccelerometerFactory.h"

// Third Party Includes

using namespace frc;

/// @brief    Find or create the factory
/// @returns  BuiltInAccelerometerFactory* pointer to the factory
DragonRoboRioAccelerometerFactory *DragonRoboRioAccelerometerFactory::m_factory = nullptr;
DragonRoboRioAccelerometerFactory *DragonRoboRioAccelerometerFactory::GetInstance()
{
    if (DragonRoboRioAccelerometerFactory::m_factory == nullptr)
    {
        DragonRoboRioAccelerometerFactory::m_factory = new DragonRoboRioAccelerometerFactory();
    }
    return DragonRoboRioAccelerometerFactory::m_factory;
}

/// @brief      Create the Builtin Accelerometer
/// @returns 	DragonBuiltinAccelerometer*
DragonBuiltinAccelerometer *DragonRoboRioAccelerometerFactory::CreateAccelerometer(RoboRioOrientation::ROBORIO_ORIENTATION orientation)
{
    if (m_accel == nullptr)
    {
        switch (orientation)
        {
        // Z Axis Up
        case RoboRioOrientation::ROBORIO_ORIENTATION::X_FORWARD_Y_LEFT:
            m_accel = new DragonBuiltinAccelerometer();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_LEFT_Y_BACKWARD:
            m_accel = new DragonRoboRioAccelXLeftYBackward();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_BACKWARD_Y_RIGHT:
            m_accel = new DragonRoboRioAccelXBackwardYRight();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_RIGHT_Y_FORWARD:
            m_accel = new DragonRoboRioAccelXRightYForward();
            break;

        // Z Axis Down
        case RoboRioOrientation::ROBORIO_ORIENTATION::X_FORWARD_Y_RIGHT:
            m_accel = new DragonRoboRioAccelXForwardYRight();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_LEFT_Y_FORWARD:
            m_accel = new DragonRoboRioAccelXLeftYForward();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_BACKWARD_Y_LEFT:
            m_accel = new DragonRoboRioAccelXBackwardYLeft();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_RIGHT_Y_BACKWARD:
            m_accel = new DragonRoboRioAccelXRightYBackward();
            break;

        // Z Axis Backward
        case RoboRioOrientation::ROBORIO_ORIENTATION::X_UP_Y_LEFT:
            m_accel = new DragonRoboRioAccelXUpYLeft();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_LEFT_Y_DOWN:
            m_accel = new DragonRoboRioAccelXLeftYDown();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_DOWN_Y_RIGHT:
            m_accel = new DragonRoboRioAccelXDownYRight();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_RIGHT_Y_UP:
            m_accel = new DragonRoboRioAccelXRightYUp();
            break;

        // Z Axis Forward
        case RoboRioOrientation::ROBORIO_ORIENTATION::X_UP_Y_RIGHT:
            m_accel = new DragonRoboRioAccelXUpYRight();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_LEFT_Y_UP:
            m_accel = new DragonRoboRioAccelXLeftYUp();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_DOWN_Y_LEFT:
            m_accel = new DragonRoboRioAccelXDownYLeft();
            break;

        case RoboRioOrientation::ROBORIO_ORIENTATION::X_RIGHT_Y_DOWN:
            m_accel = new DragonRoboRioAccelXRightYDown();
            break;

        default:
            m_accel = new DragonBuiltinAccelerometer();
            break;
        }
    }
    return m_accel;
}
DragonRoboRioAccelerometerFactory::DragonRoboRioAccelerometerFactory() : m_accel(nullptr)
{
}

DragonRoboRioAccelerometerFactory::~DragonRoboRioAccelerometerFactory()
{
    delete m_accel;
    m_accel = nullptr;
}