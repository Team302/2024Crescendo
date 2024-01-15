
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

#pragma once

// FRC includes
#include <frc/GenericHID.h>

// Team 302 includes
#include <gamepad/IDragonGamepad.h>

// forward declares
class IProfile;
class IDeadband;
class InvertAxis;
class ScaledAxis;

//==================================================================================
/// @class  AnalogAxis
/// @brief  This class handles the analog inputs on a gamepad
//==================================================================================
class AnalogAxis
{
public:
    //=========================================================================================
    /// @brief  construct the AnalogAxis object
    /// @param [in] frc::GenericHID* gamepad - gamepad to query
    /// @param [in] int axisID - id this axis maps to
    /// @param [in] bool flipped - true the axis is reversed from what is expected, false the axis
    ///         has the expected direction.
    //=========================================================================================
    AnalogAxis(
        frc::GenericHID *gamepad,
        int axisID,
        bool flipped);

    //================================================================================================
    /// @brief  Read the analog (axis) value and return it.  If the gamepad has an issue, return 0.0.
    //================================================================================================
    double GetAxisValue();

    //================================================================================================
    /// @brief  Set the deadband type
    /// @param  TeleopControlMappingEnums::AXIS_DEADBAND type - deadband option
    /// @return void
    //================================================================================================
    void SetDeadBand(
        TeleopControlMappingEnums::AXIS_DEADBAND type);

    //================================================================================================
    /// @brief  Set the axis profile (cubed, linear, etc.)
    /// @param  TeleopControlMappingEnums::AXIS_PROFILE profile - profile option
    /// @return void
    //================================================================================================
    void SetAxisProfile(
        TeleopControlMappingEnums::AXIS_PROFILE profile);

    void SetInverted(
        bool isInverted);

    //================================================================================================
    /// @brief  Set the axis scale factor (default is 1.0)
    /// @param  double scale - value greater than 0.0
    /// @return void
    //================================================================================================
    void SetAxisScaleFactor(
        double scale /// <I> - sacle factor
    );

    void DefinePerpendicularAxis(
        AnalogAxis *m_perpendicularAxis);

protected:
    //==================================================================================
    /// @brief  Returns the analog input's raw value. If there is a connection problem,
    ///         0.0 will be returned and a debug message will be written.
    /// @return double - raw axis value
    //==================================================================================
    virtual double GetRawValue();
    double GetRawInvertedValue();

    inline frc::GenericHID *GetGameePad() const { return m_gamepad; }
    inline int GetAxisID() const { return m_axis; }

private:
    frc::GenericHID *m_gamepad;
    int m_axis;
    IProfile *m_profile;
    IDeadband *m_deadband;
    ScaledAxis *m_scale;
    InvertAxis *m_inversion;
    AnalogAxis *m_secondaryAxis;
};
