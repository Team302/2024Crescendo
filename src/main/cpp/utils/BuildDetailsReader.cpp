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
#include <fstream>
#include <iostream>

// FIRST Includes
#include <frc/Filesystem.h>

// Team 302 Includes
#include <utils/BuildDetailsReader.h>

BuildDetails BuildDetailsReader::ReadBuildDetails()
{
    if (m_details.teamNumber != -1)
    {
        return m_details;
    }

    // open the file in the deploy folder when the code is running
    // std::ifstream buildDetailsFile(frc::filesystem::GetDeployDirectory() + "BuildDetails.txt");
    /// for testing only, when on robot use other line for ifstream
    std::ifstream buildDetailsFile("BuildDetails.txt");
    std::string line;
    std::string wholeFile = "";
    bool successOpening = false;

    // check if file is open
    if (buildDetailsFile.is_open())
    {
        successOpening = true;
        while (getline(buildDetailsFile, line))
        {
            wholeFile += line;
        }
    }

    buildDetailsFile.close();

    if (successOpening)
    {
        for (auto i = 0; i < m_markers.size(); i++)
        {
            std::string value = getStringBetweenMarkers(wholeFile, m_markers[i]);
            switch (i)
            {
            case 0:
                m_details.teamNumber = std::stoi(value);
                break;
            case 1:
                m_details.dateFormatted = value;
                break;
            case 2:
                m_details.date = value;
                break;
            case 3:
                m_details.branch = value;
                break;
            case 4:
                m_details.author = value;
                break;
            case 5:
                m_details.commitHash = value;
                break;
            };
        }
    }

    return m_details;
}

std::string BuildDetailsReader::getStringBetweenMarkers(std::string string, std::string marker)
{
    unsigned firstMarkerPosition = string.find(marker);
    unsigned endPositionOfFirstMarker = firstMarkerPosition + marker.length();
    unsigned lastMarkerPosition = string.find(marker, endPositionOfFirstMarker);

    return string.substr(endPositionOfFirstMarker, lastMarkerPosition - endPositionOfFirstMarker);
}