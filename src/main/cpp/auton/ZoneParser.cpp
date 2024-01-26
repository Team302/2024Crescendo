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

    AutonGrid::XGRID xgrid1 = AutonGrid::XGRID::NO_VALUE;
    AutonGrid::YGRID ygrid1 = AutonGrid::YGRID::NONE;
    AutonGrid::XGRID xgrid2 = AutonGrid::XGRID::NO_VALUE;
    AutonGrid::YGRID ygrid2 = AutonGrid::YGRID::NONE;

    // looping through the zone xml attributes to define the location of a given zone (based on 2 sets grid coordinates)

    for (xml_attribute attr = zonenode.first_attribute(); attr; attr = attr.next_attribute())
    {

        if (strcmp(attr.name(), "xgrid1") == 0)
        {
            auto val = attr.as_int();
            if (val > AutonGrid::XGRID::NO_VALUE && val < AutonGrid::XGRID::EXCEEDING_VALUE) // determine if grid coordinates are in range of the enum
            {
                xgrid1 = static_cast<AutonGrid::XGRID>(val); // casting that val to one of the enum values
            }
        }
        if (strcmp(attr.name(), "ygrid1") == 0)
        {
            auto val = attr.as_int();
            if (val > AutonGrid::YGRID::NONE && val < AutonGrid::YGRID::EXCEEDED)
            {
                ygrid1 = static_cast<AutonGrid::YGRID>(val);
            }
        }
        if (strcmp(attr.name(), "xgrid2") == 0)
        {
            auto val = attr.as_int();
            if (val > AutonGrid::XGRID::NO_VALUE && val < AutonGrid::XGRID::EXCEEDING_VALUE)
            {
                xgrid2 = static_cast<AutonGrid::XGRID>(val);
            }
        }
        if (strcmp(attr.name(), "ygrid2") == 0)
        {
            auto val = attr.as_int();
            if (val > AutonGrid::YGRID::NONE && val < AutonGrid::YGRID::EXCEEDED)
            {
                ygrid2 = static_cast<AutonGrid::YGRID>(val);
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
