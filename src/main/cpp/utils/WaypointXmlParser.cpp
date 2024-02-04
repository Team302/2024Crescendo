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

// FRC Includes
#include <frc/Filesystem.h>

// Team 302 Includes
#include "utils/WaypointXmlParser.h"
#include "utils/logging/Logger.h"
#include "utils/Waypoint2d.h"
#include "chassis/driveStates/DragonTrajectoryGenerator.h"
// Third party includes
#include <pugixml/pugixml.hpp>

using namespace pugi;
using namespace std;

WaypointXmlParser *WaypointXmlParser::m_instance = nullptr;
WaypointXmlParser *WaypointXmlParser::GetInstance()
{
    if (WaypointXmlParser::m_instance == nullptr)
    {
        WaypointXmlParser::m_instance = new WaypointXmlParser();
    }
    return WaypointXmlParser::m_instance;
}

void WaypointXmlParser::ParseWaypoints()
{
    // set the file to parse
    auto deployDir = frc::filesystem::GetDeployDirectory();
    std::string filename = deployDir + std::string("/waypoints.xml");

    map<string, DragonTrajectoryGenerator::WAYPOINTS> waypointMap;
    waypointMap[string("GRID_WALL_INTERMEDIATE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_WALL_INTERMEDIATE;
    waypointMap[string("GRID_COOP_INTERMEDIATE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_COOP_INTERMEDIATE;
    waypointMap[string("GRID_HP_INTERMEDIATE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_HP_INTERMEDIATE;
    waypointMap[string("GRID_WALL_COL_ONE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_WALL_COL_ONE;
    waypointMap[string("GRID_WALL_COL_TWO")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_WALL_COL_TWO;
    waypointMap[string("GRID_WALL_COL_THREE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_WALL_COL_THREE;
    waypointMap[string("GRID_COOP_COL_ONE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_COOP_COL_ONE;
    waypointMap[string("GRID_COOP_COL_TWO")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_COOP_COL_TWO;
    waypointMap[string("GRID_COOP_COL_THREE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_COOP_COL_THREE;
    waypointMap[string("GRID_HP_COL_ONE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_HP_COL_ONE;
    waypointMap[string("GRID_HP_COL_TWO")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_HP_COL_TWO;
    waypointMap[string("GRID_HP_COL_THREE")] = DragonTrajectoryGenerator::WAYPOINTS::GRID_HP_COL_THREE;

    bool hasError = false;

    std::vector<Waypoint2d> waypoints;

    try
    {
        // load the xml file into memory (parse it)
        xml_document doc;
        xml_parse_result result = doc.load_file(filename.c_str());

        // if it is good
        if (result)
        {
            Waypoint2d currentWaypoint;
            double blueXCoordinate = 0.0;
            double blueYCoordinate = 0.0;
            double blueRotationAngle = 0.0;
            double redXCoordinate = 0.0;
            double redYCoordinate = 0.0;
            double redRotationAngle = 0.0;

            // get the root node <waypoints>
            xml_node parent = doc.root();
            for (xml_node node = parent.first_child(); node; node = node.next_sibling())
            {
                // loop through the direct children of <waypoints> and call the appropriate parser
                for (xml_node child = node.first_child(); child; child = child.next_sibling())
                {
                    for (xml_attribute attr = child.first_attribute(); attr && !hasError; attr = attr.next_attribute())
                    {
                        if (strcmp(attr.name(), "identifier") == 0)
                        {
                            currentWaypoint.waypointIdentifier = waypointMap[attr.as_string()];
                        }
                        else if (strcmp(attr.name(), "blueXCoordinate") == 0)
                        {
                            blueXCoordinate = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "blueYCoordinate") == 0)
                        {
                            blueYCoordinate = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "blueRotation") == 0)
                        {
                            blueRotationAngle = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "redXCoordinate") == 0)
                        {
                            redXCoordinate = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "redYCoordinate") == 0)
                        {
                            redYCoordinate = attr.as_double();
                        }
                        else if (strcmp(attr.name(), "redRotation") == 0)
                        {
                            redRotationAngle = attr.as_double();
                        }
                        else
                        {
                            string msg = "unknown attribute ";
                            msg += attr.name();
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("WaypointXmlParser"), string("ParseXML"), msg);
                            hasError = true;
                        }
                    }

                    currentWaypoint.bluePose = {units::length::meter_t(blueXCoordinate), units::length::meter_t(blueYCoordinate), units::angle::degree_t(blueRotationAngle)};
                    currentWaypoint.redPose = {units::length::meter_t(redXCoordinate), units::length::meter_t(redYCoordinate), units::angle::degree_t(redRotationAngle)};

                    waypoints.emplace_back(currentWaypoint);
                }
            }
        }
        else
        {
            std::string msg = "XML [";
            msg += filename;
            msg += "] parsed with errors, attr value: [";
            msg += doc.child("prototype").attribute("attr").value();
            msg += "]";
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("WaypointXmlParser"), string("ParseXML (1) "), msg);

            msg = "Error description: ";
            msg += result.description();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("WaypointXmlParser"), string("ParseXML (2) "), msg);

            msg = "Error offset: ";
            msg += result.offset;
            msg += " error at ...";
            msg += filename;
            msg += result.offset;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("WaypointXmlParser"), string("ParseXML (3) "), msg);
        }
    }
    catch (const std::exception &e)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("WaypointXmlParser"), string("ParseXML"), string("Error thrown while parsing Waypoints.xml"));
    }

    if (!hasError)
    {
        m_waypoints = waypoints;
    }
}
