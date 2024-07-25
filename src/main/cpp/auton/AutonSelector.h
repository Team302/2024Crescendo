
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

#pragma once

#include <string>
#include <vector>
#include <frc/smartdashboard/SendableChooser.h>

class AutonSelector
{

public:
	//---------------------------------------------------------------------
	// Method: 		<<constructor>>
	// Description: This creates this object and reads the auto script (CSV)
	//  			files and displays a list on the dashboard.
	//---------------------------------------------------------------------
	AutonSelector();

	//---------------------------------------------------------------------
	// Method: 		<<destructor>>
	// Description: default cleanup
	//---------------------------------------------------------------------
	virtual ~AutonSelector() = default;
	std::string GetSelectedAutoFile();
	std::string GetAlianceColor();
	std::string GetStartPos();
	std::string GetNumofPiecesinautonWing();
	std::string GetNumofPiecesinautonCenter();
	//---------------------------------------------------------------------
	// Method: 		GetSelectedAutoFile
	// Description: This returns the selected auton file to run.  If it
	//  			returns "Do Nothing", it is indicating no auton should
	//				be run.
	// Returns:		std::string			auton file to run
	//---------------------------------------------------------------------

private:
	//---------------------------------------------------------------------
	// Method: 		PutChoicesOnDashboard
	// Description: This puts the list of files in the m_csvFiles attribute
	//				up on the dashboard for selection.
	// Returns:		void
	//---------------------------------------------------------------------

	void PutChoicesOnDashboard();
	bool FileExists(const std::string &name);
	bool FileValid(const std::string &name);
	// Attributues
	frc::SendableChooser<std::string> m_startposchooser;
	frc::SendableChooser<std::string> m_numofgamepiecewing;
	frc::SendableChooser<std::string> m_numofgamepiececenter;
};
