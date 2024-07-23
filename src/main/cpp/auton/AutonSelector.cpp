
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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

// Includes
#include <string>
#include <vector>
#include <sys/stat.h>
#include <fstream>

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Filesystem.h>
#include "networktables/NetworkTableInstance.h"

// Team302 includes
#include "auton/AutonSelector.h"
#include "utils/logging/Logger.h"
#include "utils/FMSData.h"

#include <pugixml/pugixml.hpp>

using namespace std;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
AutonSelector::AutonSelector()
{
	PutChoicesOnDashboard();
}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(frc::filesystem::GetDeployDirectory());
	autonfile += std::string("/auton/");
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetNumofPiecesinautonWing();
	autonfile += GetNumofPiecesinautonCenter();
	autonfile += std::string(".xml");

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("auton"), string("file"), autonfile);

	auto table = nt::NetworkTableInstance::GetDefault().GetTable("auton file");

	table.get()->PutString("determined name", autonfile);


	bool fileExists = FileExists(autonfile);
	bool fileValid =  FileValid(autonfile);

	table.get()->PutBoolean("File Exists", fileExists);
	table.get()->PutBoolean("File Valid", fileValid);

	if (!fileExists || !fileValid)
	{
		autonfile = frc::filesystem::GetDeployDirectory();
		autonfile += std::string("/auton/");
		autonfile += GetAlianceColor();
		autonfile += ("DefaultFile.xml");	
	}

	table.get()->PutString("actual file", autonfile);

	return autonfile;
}

bool AutonSelector::FileExists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

bool AutonSelector::FileValid(const std::string &name)
{

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(name.c_str());
	if (result)
	{
		return true;
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Description ") + name, result.description());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Offset ") + name, result.offset);
	return false;
}

string AutonSelector::GetAlianceColor()
{
	if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
	{
		return std::string("Red");
	}
	else
	{
		return std::string("Blue");
	}
}

string AutonSelector::GetStartPos()
{
	return m_startposchooser.GetSelected();
}

string AutonSelector::GetNumofPiecesinautonWing()
{
	return m_numofgamepiecewing.GetSelected();
}

string AutonSelector::GetNumofPiecesinautonCenter()
{
	return m_numofgamepiececenter.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{
	m_startposchooser.AddOption("Amp", "Amp");
	m_startposchooser.AddOption("Subwoofer", "Subwoofer");
	m_startposchooser.AddOption("Podium", "Podium");
	m_startposchooser.AddOption("Wide", "Wide");
	m_startposchooser.SetDefaultOption("Subwoofer", "Subwoofer");
	frc::SmartDashboard::PutData("StartPos", &m_startposchooser);

	m_numofgamepiecewing.AddOption("0", "Wing0");
	m_numofgamepiecewing.AddOption("1", "Wing1");
	m_numofgamepiecewing.AddOption("2", "Wing2");
	m_numofgamepiecewing.AddOption("3", "Wing3");
	m_numofgamepiecewing.SetDefaultOption("0", "Wing0");
	frc::SmartDashboard::PutData("NumofWingpcs", &m_numofgamepiecewing);

	m_numofgamepiececenter.AddOption("0", "Center0");
	m_numofgamepiececenter.AddOption("1", "Center1");
	m_numofgamepiececenter.AddOption("2", "Center2");
	m_numofgamepiececenter.AddOption("3", "Center3");
	m_numofgamepiececenter.AddOption("4", "Center4");
	m_numofgamepiececenter.AddOption("5", "Center5");
	m_numofgamepiececenter.SetDefaultOption("0", "Center0");
	frc::SmartDashboard::PutData("NumofCenterpcs", &m_numofgamepiececenter);
}
