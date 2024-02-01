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
#include <string>

#include <frc/Filesystem.h>

#include "auton/AutonGrid.h"

#include <auton/ZoneParams.h>
#include <auton/ZoneParser.h>
#include "utils/logging/Logger.h"
#include <pugixml/pugixml.hpp>
using namespace std;
using namespace pugi;

ZoneParams *ZoneParser::ParseXML(xml_node zonenode)
{
    ZoneParams *zone = nullptr;
    auto hasError = false;

    static std::map<std::string, AutonGrid::XGRID> X_xmlStringToGridEnumMap{
        {"X_A", AutonGrid::XGRID::X_A},
        {"X_B", AutonGrid::XGRID::X_B},
        {"X_C", AutonGrid::XGRID::X_C},
        {"X_D", AutonGrid::XGRID::X_D},
        {"X_E", AutonGrid::XGRID::X_E},
        {"X_F", AutonGrid::XGRID::X_F},
        {"X_G", AutonGrid::XGRID::X_G},
        {"X_H", AutonGrid::XGRID::X_H},
        {"X_I", AutonGrid::XGRID::X_I},
        {"X_J", AutonGrid::XGRID::X_J},
        {"X_K", AutonGrid::XGRID::X_K},
        {"X_L", AutonGrid::XGRID::X_L},
        {"X_M", AutonGrid::XGRID::X_M},
        {"X_N", AutonGrid::XGRID::X_N},
        {"X_O", AutonGrid::XGRID::X_O},
        {"X_P", AutonGrid::XGRID::X_P},
        {"X_Q", AutonGrid::XGRID::X_Q},
        {"X_R", AutonGrid::XGRID::X_R},
        {"X_S", AutonGrid::XGRID::X_S},
        {"X_T", AutonGrid::XGRID::X_T},
        {"X_U", AutonGrid::XGRID::X_U},
        {"X_V", AutonGrid::XGRID::X_V},
        {"X_W", AutonGrid::XGRID::X_W},
        {"X_X", AutonGrid::XGRID::X_X},
        {"X_Y", AutonGrid::XGRID::X_Y},
        {"X_Z", AutonGrid::XGRID::X_Z},
        {"X_AA", AutonGrid::XGRID::X_AA}};
    static std::map<std::string, AutonGrid::YGRID> Y_xmlStringToGridEnumMap{
        {"Y_1", AutonGrid::YGRID::Y_1},
        {"Y_2", AutonGrid::YGRID::Y_2},
        {"Y_3", AutonGrid::YGRID::Y_3},
        {"Y_4", AutonGrid::YGRID::Y_4},
        {"Y_5", AutonGrid::YGRID::Y_5},
        {"Y_6", AutonGrid::YGRID::Y_6},
        {"Y_7", AutonGrid::YGRID::Y_7},
        {"Y_8", AutonGrid::YGRID::Y_8},
        {"Y_9", AutonGrid::YGRID::Y_9},
        {"Y_10", AutonGrid::YGRID::Y_10},
        {"Y_11", AutonGrid::YGRID::Y_11},
        {"Y_12", AutonGrid::YGRID::Y_12},
        {"Y_13", AutonGrid::YGRID::Y_13},
        {"Y_14", AutonGrid::YGRID::Y_14},
        {"Y_15", AutonGrid::YGRID::Y_15},
        {"Y_16", AutonGrid::YGRID::Y_16},
        {"Y_17", AutonGrid::YGRID::Y_17},
        {"Y_18", AutonGrid::YGRID::Y_18},
        {"Y_19", AutonGrid::YGRID::Y_19},
        {"Y_20", AutonGrid::YGRID::Y_20},
        {"Y_21", AutonGrid::YGRID::Y_21},
        {"Y_22", AutonGrid::YGRID::Y_22},
        {"Y_23", AutonGrid::YGRID::Y_23},
        {"Y_24", AutonGrid::YGRID::Y_24},
        {"Y_25", AutonGrid::YGRID::Y_25},
        {"Y_26", AutonGrid::YGRID::Y_26},
        {"Y_27", AutonGrid::YGRID::Y_27},
        {"Y_28", AutonGrid::YGRID::Y_28},
        {"Y_29", AutonGrid::YGRID::Y_29},
        {"Y_30", AutonGrid::YGRID::Y_30},
        {"Y_31", AutonGrid::YGRID::Y_31},
        {"Y_32", AutonGrid::YGRID::Y_32},
        {"Y_33", AutonGrid::YGRID::Y_33},
        {"Y_34", AutonGrid::YGRID::Y_34},
        {"Y_35", AutonGrid::YGRID::Y_35},
        {"Y_36", AutonGrid::YGRID::Y_36},
        {"Y_37", AutonGrid::YGRID::Y_37},
        {"Y_38", AutonGrid::YGRID::Y_38},
        {"Y_39", AutonGrid::YGRID::Y_39},
        {"Y_40", AutonGrid::YGRID::Y_40},
        {"Y_41", AutonGrid::YGRID::Y_41},
        {"Y_42", AutonGrid::YGRID::Y_42},
        {"Y_43", AutonGrid::YGRID::Y_43},
        {"Y_44", AutonGrid::YGRID::Y_44},
        {"Y_45", AutonGrid::YGRID::Y_45},
        {"Y_46", AutonGrid::YGRID::Y_46},
        {"Y_47", AutonGrid::YGRID::Y_47},
        {"Y_48", AutonGrid::YGRID::Y_48},
        {"Y_49", AutonGrid::YGRID::Y_49},
        {"Y_50", AutonGrid::YGRID::Y_50},
        {"Y_51", AutonGrid::YGRID::Y_51},
        {"Y_52", AutonGrid::YGRID::Y_52},
        {"Y_53", AutonGrid::YGRID::Y_53},
        {"Y_54", AutonGrid::YGRID::Y_54}};

    static std::map<std::string, RobotElementNames::STATE_NOTE_MANAGER_USAGE> xmlStringToSTATE_NOTE_MANAGER_USAGEEnumMap{
        {"UNKNOWN_STATE_NOTE_MANAGER", RobotElementNames::STATE_NOTE_MANAGER_USAGE::UNKNOWN_STATE_NOTE_MANAGER},
        {"NOTE_MANAGER_OFF", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_OFF},
        {"NOTE_MANAGER_READY", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_READY},
        {"NOTE_MANAGER_FEEDER_INTAKE", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_FEEDER_INTAKE},
        {"NOTE_MANAGER_EXPEL", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_EXPEL},
        {"NOTE_MANAGER_PLACER_INTAKE", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PLACER_INTAKE},
        {"NOTE_MANAGER_HOLD_FEEDER_FRONT", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_HOLD_FEEDER_FRONT},
        {"NOTE_MANAGER_HOLD_FEEDER_BACK", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_HOLD_FEEDER_BACK},
        {"NOTE_MANAGER_INTAKE_TO_FEEDER", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_INTAKE_TO_FEEDER},
        {"NOTE_MANAGER_LAUNCHER_TO_PLACER_FRONT", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_LAUNCHER_TO_PLACER_FRONT},
        {"NOTE_MANAGER_LAUNCHER_TO_PLACER_BACK", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_LAUNCHER_TO_PLACER_BACK},
        {"NOTE_MANAGER_HOLD_FEEDER", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_HOLD_FEEDER},
        {"NOTE_MANAGER_READY_AUTO_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_READY_AUTO_LAUNCH},
        {"NOTE_MANAGER_READY_MANUAL_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_READY_MANUAL_LAUNCH},
        {"NOTE_MANAGER_PASS", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PASS},
        {"NOTE_MANAGER_AUTO_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_AUTO_LAUNCH},
        {"NOTE_MANAGER_MANUAL_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_MANUAL_LAUNCH},
        {"NOTE_MANAGER_READY_ODOMETRY_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_READY_ODOMETRY_LAUNCH},
        {"NOTE_MANAGER_AUTO_LAUNCH_ODOMETRY", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_AUTO_LAUNCH_ODOMETRY},
        {"NOTE_MANAGER_HOLD_PLACER_FRONT", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_HOLD_PLACER_FRONT},
        {"NOTE_MANAGER_HOLD_PLACER_BACK", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_HOLD_PLACER_BACK},
        {"NOTE_MANAGER_INTAKE_TO_PLACER", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_INTAKE_TO_PLACER},
        {"NOTE_MANAGER_PREPARE_PLACE_AMP", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PREPARE_PLACE_AMP},
        {"NOTE_MANAGER_PREPARE_PLACE_TRAP", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PREPARE_PLACE_TRAP},
        {"NOTE_MANAGER_PLACE_AMP", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PLACE_AMP},
        {"NOTE_MANAGER_PLACE_TRAP", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PLACE_TRAP},
        {"NOTE_MANAGER_PLACER_TO_LAUNCHER_FRONT", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PLACER_TO_LAUNCHER_FRONT},
        {"NOTE_MANAGER_PLACER_TO_LAUNCHER_BACK", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_PLACER_TO_LAUNCHER_BACK},
        {"NOTE_MANAGER_BACKUP_MANUAL_LAUNCH", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_BACKUP_MANUAL_LAUNCH},
        {"NOTE_MANAGER_BACKUP_MANUAL_PLACE", RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_BACKUP_MANUAL_PLACE},
        {"MAX_STATE_NOTE_MANAGER", RobotElementNames::STATE_NOTE_MANAGER_USAGE::MAX_STATE_NOTE_MANAGER}};

    static std::map<std::string, RobotElementNames::STATE_NOTE_MANAGER_USAGE> xmlStringToChassisOptionEnumMap{

    };

    AutonGrid::XGRID xgrid1 = AutonGrid::XGRID::NO_VALUE;
    AutonGrid::YGRID ygrid1 = AutonGrid::YGRID::NONE;
    AutonGrid::XGRID xgrid2 = AutonGrid::XGRID::NO_VALUE;
    AutonGrid::YGRID ygrid2 = AutonGrid::YGRID::NONE;
    RobotElementNames::STATE_NOTE_MANAGER_USAGE noteChosenOption = RobotElementNames::STATE_NOTE_MANAGER_USAGE::NOTE_MANAGER_OFF;

    // looping through the zone xml attributes to define the location of a given zone (based on 2 sets grid coordinates)

    for (xml_attribute attr = zonenode.first_attribute(); attr; attr = attr.next_attribute())
    {

        if (strcmp(attr.name(), "xgrid1") == 0)
        {
            auto itr = X_xmlStringToGridEnumMap.find(attr.value());
            if (itr != X_xmlStringToGridEnumMap.end())
            {
                xgrid1 = itr->second;
            }
            else
            {
                hasError = true;
            }
        }
        else if (strcmp(attr.name(), "ygrid1") == 0)
        {
            auto itr = Y_xmlStringToGridEnumMap.find(attr.value());
            if (itr != Y_xmlStringToGridEnumMap.end())
            {
                ygrid1 = itr->second;
            }
            else
            {
                hasError = true;
            }
        }
        else if (strcmp(attr.name(), "xgrid2") == 0)
        {
            auto itr = X_xmlStringToGridEnumMap.find(attr.value());
            if (itr != X_xmlStringToGridEnumMap.end())
            {
                xgrid2 = itr->second;
            }
            else
            {
                hasError = true;
            }
        }
        else if (strcmp(attr.name(), "ygrid2") == 0)
        {
            auto itr = Y_xmlStringToGridEnumMap.find(attr.value());
            if (itr != Y_xmlStringToGridEnumMap.end())
            {
                ygrid2 = itr->second;
            }
            else
            {
                hasError = true;
            }
        }
        else if (strcmp(attr.name(), "chassisOption") == 0)
        {
            if (strcmp(attr.value(), "visiondrivenote") == 0)
            {
            }
            else if (strcmp(attr.value(), "visiondrivespeaker") == 0)
            {
            }
            else if (strcmp(attr.value(), "none") == 0)
            {
            }
        }
        else if (strcmp(attr.name(), "noteOption") == 0)
        {
            auto itr = xmlStringToSTATE_NOTE_MANAGER_USAGEEnumMap.find(attr.value());
            if (itr != xmlStringToSTATE_NOTE_MANAGER_USAGEEnumMap.end())
            {
                noteChosenOption = itr->second;
            }
            else
            {
                hasError = true;
            }
        }
        else if (strcmp(attr.name(), "avoidOption") == 0)
        {
            if (strcmp(attr.value(), "podium") == 0)
            {
            }
            else if (strcmp(attr.value(), "robotcollision") == 0)
            {
            }
            else if (strcmp(attr.value(), "none") == 0)
            {
            }
        }
    }

    if (!hasError) // if no error returns the zone parameters
    {
        return (new ZoneParams(xgrid1,
                               ygrid1,
                               xgrid2,
                               ygrid2));
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ZoneParser"), string("ParseXML"), string("Has Error"));

    return nullptr; // if error, return nullptr
}
