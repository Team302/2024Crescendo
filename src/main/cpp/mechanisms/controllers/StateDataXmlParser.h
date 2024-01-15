
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

#pragma once
#include <vector>

#include "mechanisms/MechanismTypes.h"
#include "mechanisms/controllers/MechanismTargetData.h"

//========================================================================================================
/// StateDataXmlParser.h
//========================================================================================================
///
/// File Description:
///     Top-level XML parsing file for the state data.  This definition will construct objects to help
///     set the state targets and control modes.
///
///     This parsing leverages the 3rd party Open Source Pugixml library (https://pugixml.org/).
///
///     The state definition XML files are in:  /home/lvuser/config/states/XXX.xml where the XXX
///     is the mechanism name.
///
//========================================================================================================
class StateDataXmlParser
{
public:
    StateDataXmlParser() = default;
    virtual ~StateDataXmlParser() = default;

    /// @brief      Parse a mechanismState.xml file
    /// @param [in] MechanismTypes::MECHANISM_TYPE  - mechanism that the states are for
    /// @return     state data
    std::vector<MechanismTargetData *> ParseXML(
        MechanismTypes::MECHANISM_TYPE mechanism);
};
