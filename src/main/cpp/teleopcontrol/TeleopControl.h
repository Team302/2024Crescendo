
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

// C++ Includes
#include <array>
#include <memory>
#include <utility>
#include <vector>

// FRC includes
#include <frc/DriverStation.h>

// Team 302 includes
#include <gamepad/IDragonGamepad.h>
#include <teleopcontrol/TeleopControlAxis.h>
#include <teleopcontrol/TeleopControlButton.h>
#include <teleopcontrol/TeleopControlFunctions.h>
#include <teleopcontrol/TeleopControlMappingEnums.h>
#include "utils/logging/LoggableItem.h"

// third part
#include <RobinHood/robin_hood.h>

class TeleopControl : LoggableItem
{
public:
    //----------------------------------------------------------------------------------
    // Method:      GetInstance
    // Description: If there isn't an instance of this class, it will create one.  The
    //              single class instance will be returned.
    // Returns:     OperatorInterface*  instance of this class
    //----------------------------------------------------------------------------------
    static TeleopControl *GetInstance();

    //------------------------------------------------------------------
    // Method:      GetAxisValue
    // Description: Reads the joystick axis, removes any deadband (small
    //              value) and then scales as requested.
    // Returns:     double   -  scaled axis value
    //------------------------------------------------------------------
    double GetAxisValue(
        TeleopControlFunctions::FUNCTION axis // <I> - axis number to update
    );

    //------------------------------------------------------------------
    // Method:      GetRawButton
    // Description: Reads the button value.  Also allows POV, bumpers,
    //              and triggers to be treated as buttons.
    // Returns:     bool   -  scaled axis value
    //------------------------------------------------------------------
    bool IsButtonPressed(
        TeleopControlFunctions::FUNCTION button // <I> - button number to query
    );

    void SetRumble(
        TeleopControlFunctions::FUNCTION button, // <I> - controller with this function
        bool leftRumble,                         // <I> - rumble left
        bool rightRumble                         // <I> - rumble right
    );

    void SetRumble(
        int controller,  // <I> - controller to rumble
        bool leftRumble, // <I> - rumble left
        bool rightRumble // <I> - rumble right
    );

    void LogInformation() override;

private:
    //----------------------------------------------------------------------------------
    // Method:      OperatorInterface <<constructor>>
    // Description: This will construct and initialize the object
    //----------------------------------------------------------------------------------
    TeleopControl();

    //----------------------------------------------------------------------------------
    // Method:      ~OperatorInterface <<destructor>>
    // Description: This will clean up the object
    //----------------------------------------------------------------------------------
    virtual ~TeleopControl() = default;

    void Initialize();
    bool IsInitialized() const;

    void InitializeControllers();
    void InitializeController(int port);
    void InitializeAxes(int port);
    void InitializeButtons(int port);

    std::vector<TeleopControlFunctions::FUNCTION> GetAxisFunctionsOnController(int controller);
    std::vector<TeleopControlFunctions::FUNCTION> GetButtonFunctionsOnController(int controller);

    //------------------------------------------------------------------
    // Method:      SetScaleFactor
    // Description: Allow the range of values to be set smaller than
    //              -1.0 to 1.0.  By providing a scale factor between 0.0
    //              and 1.0, the range can be made smaller.  If a value
    //              outside the range is provided, then the value will
    //              be set to the closest bounding value (e.g. 1.5 will
    //              become 1.0)
    // Returns:     void
    //------------------------------------------------------------------
    void SetAxisScaleFactor(
        TeleopControlFunctions::FUNCTION axis, // <I> - axis number to update
        double scaleFactor                     // <I> - scale factor used to limit the range
    );

    void SetDeadBand(
        TeleopControlFunctions::FUNCTION axis,
        TeleopControlMappingEnums::AXIS_DEADBAND deadband);

    //------------------------------------------------------------------
    // Method:      SetAxisProfile
    // Description: Sets the axis profile for the specifed axis
    // Returns:     void
    //------------------------------------------------------------------
    void SetAxisProfile(
        TeleopControlFunctions::FUNCTION axis,          // <I> - axis number to update
        TeleopControlMappingEnums::AXIS_PROFILE profile // <I> - profile to use
    );

    std::pair<IDragonGamepad *, TeleopControlMappingEnums::AXIS_IDENTIFIER> GetAxisInfo(
        TeleopControlFunctions::FUNCTION function // <I> - controller with this function
    );

    std::pair<IDragonGamepad *, TeleopControlMappingEnums::BUTTON_IDENTIFIER> GetButtonInfo(
        TeleopControlFunctions::FUNCTION function // <I> - controller with this function
    );

    //----------------------------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------------------------
    static TeleopControl *m_instance; // Singleton instance of this class

    std::array<IDragonGamepad *, frc::DriverStation::kJoystickPorts> m_controller;

    int m_numControllers;
};
