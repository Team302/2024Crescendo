//====================================================================================================================================================
// ControlDataXmlParser.h
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
#include <cstring>
#include <map>
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/controllers/ControlData.h"
#include <mechanisms/controllers/ControlModes.h>
#include "utils/logging/Logger.h"
#include <mechanisms/controllers/ControlDataXmlParser.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace pugi;
using namespace std;

/// @brief Parse MechanismData XML element
/// @param [in] pugi::xml_node mechanism data node
/// @return ControlData* mechanism control data
ControlData *ControlDataXmlParser::ParseXML(
    xml_node PIDNode)
{
    // initialize output
    ControlData *data = nullptr;

    // initialize attributes to default values
    string identifier;
    ControlModes::CONTROL_TYPE mode = ControlModes::CONTROL_TYPE::PERCENT_OUTPUT;
    ControlModes::CONTROL_RUN_LOCS server = ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER;

    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;
    ControlData::FEEDFORWARD_TYPE fType = ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE;
    double izone = 0.0;
    double maxAccel = 0.0;
    double cruiseVel = 0.0;
    double peak = 1.0;
    double nominal = 0.0;

    map<string, ControlModes::CONTROL_TYPE> modeMap;
    modeMap[string("PERCENT_OUTPUT")] = ControlModes::CONTROL_TYPE::PERCENT_OUTPUT;
    modeMap[string("VELOCITY_INCH")] = ControlModes::CONTROL_TYPE::VELOCITY_INCH;
    modeMap[string("VELOCITY_DEGREES")] = ControlModes::CONTROL_TYPE::VELOCITY_DEGREES;
    modeMap[string("VELOCITY_RPS")] = ControlModes::CONTROL_TYPE::VELOCITY_RPS;
    modeMap[string("VOLTAGE")] = ControlModes::CONTROL_TYPE::VOLTAGE;
    modeMap[string("TRAPEZOID")] = ControlModes::CONTROL_TYPE::TRAPEZOID;
    modeMap[string("PERCENT_OUTPUT")] = ControlModes::CONTROL_TYPE::PERCENT_OUTPUT;
    modeMap[string("POSITION_DEGREES")] = ControlModes::CONTROL_TYPE::POSITION_DEGREES;
    modeMap[string("POSITION_INCH")] = ControlModes::CONTROL_TYPE::POSITION_INCH;
    modeMap[string("POSITION_ABS_TICKS")] = ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE;

    map<string, ControlModes::CONTROL_RUN_LOCS> serverMap;
    serverMap[string("MOTORCONTROLLER")] = ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER;
    serverMap[string("ROBORIO")] = ControlModes::CONTROL_RUN_LOCS::ROBORIO;

    map<string, ControlData::FEEDFORWARD_TYPE> ftypeMap;
    ftypeMap[string("VOLTAGE")] = ControlData::FEEDFORWARD_TYPE::VOLTAGE;
    ftypeMap[string("TORQUE_CURRENT")] = ControlData::FEEDFORWARD_TYPE::TORQUE_CURRENT;
    ftypeMap[string("DUTY_CYCLE")] = ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE;

    bool hasError = false;

    // parse/validate xml
    for (pugi::xml_attribute attr = PIDNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        if (strcmp(attr.name(), "identifier") == 0)
        {
            identifier = string(attr.value());
        }
        else if (strcmp(attr.name(), "mode") == 0)
        {
            auto it = modeMap.find(string(attr.value()));
            if (it != modeMap.end())
            {
                mode = it->second;
            }
        }
        else if (strcmp(attr.name(), "constrolServer") == 0)
        {
            auto itr = serverMap.find(string(attr.value()));
            if (itr != serverMap.end())
            {
                server = itr->second;
            }
        }
        else if (strcmp(attr.name(), "proportional") == 0)
        {
            p = attr.as_double();
        }
        else if (strcmp(attr.name(), "integral") == 0)
        {
            i = attr.as_double();
        }
        else if (strcmp(attr.name(), "derivative") == 0)
        {
            d = attr.as_double();
        }
        else if (strcmp(attr.name(), "feedforward") == 0)
        {
            f = attr.as_double();
        }
        else if (strcmp(attr.name(), "feedforwardType"))
        {
            auto itr = ftypeMap.find(string(attr.value()));
            if (itr != ftypeMap.end())
            {
                fType = itr->second;
            }
        }
        else if (strcmp(attr.name(), "izone") == 0)
        {
            izone = attr.as_double();
        }
        else if (strcmp(attr.name(), "maxacceleration") == 0)
        {
            maxAccel = attr.as_double();
        }
        else if (strcmp(attr.name(), "cruisevelocity") == 0)
        {
            cruiseVel = attr.as_double();
        }
        else if (strcmp(attr.name(), "peak") == 0)
        {
            peak = attr.as_double();
        }
        else if (strcmp(attr.name(), "nominal") == 0)
        {
            nominal = attr.as_double();
        }
        else
        {
            string msg = string("invalid attribute ");
            msg += attr.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ControlDataXmlParser"), string("ParseXML"), msg);
            hasError = true;
        }
    }
    if (!hasError)
    {
        data = new ControlData(mode, server, identifier, p, i, d, f, fType, izone, maxAccel, cruiseVel, peak, nominal, false);
    }
    return data;
}
