
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
#include <cmath>
#include <string>

// FRC includes
#include <frc/GenericHID.h>

// Team 302 includes
#include <gamepad/axis/AnalogAxis.h>
#include <gamepad/axis/CubedProfile.h>
#include <gamepad/axis/DeadbandValue.h>
#include <gamepad/axis/InvertAxis.h>
#include <gamepad/axis/LinearProfile.h>
#include <gamepad/axis/NoDeadbandValue.h>
#include <gamepad/axis/ScaledAxis.h>
#include <gamepad/axis/ScaledDeadbandValue.h>
#include <gamepad/axis/SquaredProfile.h>
#include <gamepad/IDragonGamepad.h>
#include "utils/logging/Logger.h"

// Third Party Includes
#include <units/dimensionless.h>

using namespace std;
using namespace frc;

//=========================================================================================
/// @brief  construct the AnalogAxis object
/// @param [in] frc::GenericHID* gamepad - gamepad to query
/// @param [in] int axisID - id this axis maps to
/// @param [in] bool flipped - true the axis is reversed from what is expected, false the axis
///         has the expected direction.
//=========================================================================================
AnalogAxis::AnalogAxis(
    GenericHID *gamepad,
    int axisID,
    bool flipAxis) : m_gamepad(gamepad),
                     m_axis(axisID),
                     m_profile(LinearProfile::GetInstance()),
                     m_deadband(NoDeadbandValue::GetInstance()),
                     m_scale(new ScaledAxis()),
                     m_inversion(new InvertAxis()),
                     m_secondaryAxis(nullptr)
{
    m_inversion->SetInverted(flipAxis);
}

//================================================================================================
/// @brief  Read the analog (axis) value and return it.  If the gamepad has an issue, return 0.0.
/// @return double axis value after applying the deadband, profile and scaling operations
//================================================================================================

double AnalogAxis::GetAxisValue()
{
    if (m_gamepad != nullptr)
    {
        auto value = GetRawValue();
        m_deadband->ApplyDeadband(value);
        m_profile->ApplyProfile(value);
        m_scale->Scale(value);
        m_inversion->ApplyInversion(value);

        if (m_secondaryAxis != nullptr)
        {
            auto raw1 = GetRawInvertedValue();
            auto raw2 = m_secondaryAxis->GetRawInvertedValue();
            if (abs(raw2) > 0.05)
            {
                auto angle = atan2(raw2, raw1);
                value *= abs(cos(angle));
            }
        }
        return value;
    }
    return 0.0;
}

//================================================================================================
/// @brief  Set the deadband type
/// @param  TeleopControlMappingEnums::AXIS_DEADBAND type - deadband option
/// @return void
//================================================================================================
void AnalogAxis::SetDeadBand(
    TeleopControlMappingEnums::AXIS_DEADBAND type /// <I> - deadband option
)
{
    switch (type)
    {
    case TeleopControlMappingEnums::AXIS_DEADBAND::NONE:
        m_deadband = NoDeadbandValue::GetInstance();
        break;

    case TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND:
        m_deadband = DeadbandValue::GetInstance();
        break;

    case TeleopControlMappingEnums::AXIS_DEADBAND::APPLY_SCALED_DEADBAND:
        m_deadband = ScaledDeadbandValue::GetInstance();
        break;

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("AnalogAxis::AnalogAxis::SetDeadBand"), string("invalid deadband"), string("true"));
        break;
    }
}

//================================================================================================
/// @brief  Set the axis profile (cubed, linear, etc.)
/// @param  TeleopControlMappingEnums::AXIS_PROFILE profile - profile option
/// @return void
//================================================================================================
void AnalogAxis::SetAxisProfile(
    TeleopControlMappingEnums::AXIS_PROFILE profile /// <I> - axis profile
)
{
    switch (profile)
    {
    case TeleopControlMappingEnums::AXIS_PROFILE::CUBED:
        m_profile = CubedProfile::GetInstance();
        break;

    case TeleopControlMappingEnums::AXIS_PROFILE::SQUARED:
        m_profile = SquaredProfile::GetInstance();
        break;

    case TeleopControlMappingEnums::AXIS_PROFILE::LINEAR:
        m_profile = LinearProfile::GetInstance();
        break;

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("AnalogAxis::SetAxisProfile"), string("invalid profile"), string("true"));
        break;
    }
}

//================================================================================================
/// @brief  Set the axis scale factor (default is 1.0)
/// @param  double scale - value greater than 0.0
/// @return void
//================================================================================================
void AnalogAxis::SetAxisScaleFactor(
    double scale /// <I> - scale factor - must be positive number
)
{
    if (m_scale != nullptr)
    {
        m_scale->SetScaleFactor(scale);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("AnalogAxis::SetAxisScaleFactor"), string("no scale"), string("true"));
    }
}

void AnalogAxis::SetInverted(
    bool isInverted)
{
    if (m_scale != nullptr)
    {
        m_inversion->SetInverted(isInverted);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("AnalogAxis::SetInverted"), string("no scale"), string("true"));
    }
}
//==================================================================================
/// @brief  Returns the analog input's raw value. If there is a connection problem,
///         0.0 will be returned and a debug message will be written.
/// @return double - raw axis value
//==================================================================================
double AnalogAxis::GetRawValue()
{
    if (m_gamepad != nullptr)
    {
        return m_gamepad->GetRawAxis(m_axis);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("AnalogAxis::GetRawValue"), string("gamepad is nullptr"), string("true"));
    }

    return 0.0;
}

double AnalogAxis::GetRawInvertedValue()
{
    auto val = GetRawValue();
    m_deadband->ApplyDeadband(val);
    m_inversion->ApplyInversion(val);
    return val;
}

void AnalogAxis::DefinePerpendicularAxis(
    AnalogAxis *axis)
{
    m_secondaryAxis = axis;
}
