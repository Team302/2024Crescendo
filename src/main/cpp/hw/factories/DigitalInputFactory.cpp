
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
#include <string>

// FRC includes

// Team 302 includes
#include "hw/DragonDigitalInput.h"
#include "hw/factories/DigitalInputFactory.h"
#include "configs/RobotElementNames.h"

// Third Party Includes
#include "units/time.h"
using namespace std;

/// @brief    Find or create the Digital input factory
/// @returns  DigitalInputFactory* pointer to the factory
DigitalInputFactory *DigitalInputFactory::m_factory = nullptr;
DigitalInputFactory *DigitalInputFactory::GetFactory()
{
    if (DigitalInputFactory::m_factory == nullptr)
    {
        DigitalInputFactory::m_factory = new DigitalInputFactory();
    }
    return DigitalInputFactory::m_factory;
}

/// @brief   Create the requested Digital input
/// @returns DragonDigitalInput*     pointer to the digital input or nullptr if it fails
DragonDigitalInput *DigitalInputFactory::CreateInput(
    string networkTableName,
    RobotElementNames::DIGITAL_INPUT_USAGE type,
    int digitalID,
    bool reversed,
    units::time::second_t debounceTime)
{
    DragonDigitalInput *sensor = nullptr;
    switch (type)
    {

    default:
        break;
    }
    return sensor;
}

/// @brief   Get the requested Digital input
/// @returns DragonDigitalInput*     pointer to the digital input or nullptr if it doesn't exist
DragonDigitalInput *DigitalInputFactory::GetInput(
    RobotElementNames::DIGITAL_INPUT_USAGE type)
{
    DragonDigitalInput *sensor = nullptr;
    switch (type)
    {

    default:
        break;
    }
    return sensor;
}

DigitalInputFactory::DigitalInputFactory()
{
}

DigitalInputFactory::~DigitalInputFactory()
{
}