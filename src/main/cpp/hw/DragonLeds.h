
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
#include <frc/AddressableLED.h>
#include <vector>
#include <array>

class DragonLeds
{
public:
	enum Colors
	{
		RED,
		GREEN,
		BLUE,
		PURPLE,
		YELLOW,
		AZUL,
		BLACK,
		WHITE,
		MAX_STATE
	};

	std::vector<frc::AddressableLED::LEDData> m_ledBuffer;

	void Initialize(int PWMport, int numLeds);
	bool IsInitialized() const;

	void commitLedData();

	void setOn();
	void setOff();

	void setBufferAllLEDsColor(std::array<int, 3> color);
	void setBufferAllLEDsAlternatingColor(std::array<int, 3> color1, std::array<int, 3> color2);
	void setBufferAllLEDsBlack();
	void setBufferAllLEDsRainbow();
	void setSpecificLED(int id, std::array<int, 3> color);

	std::array<int, 3> getColorValues(Colors c);

	static DragonLeds *GetInstance();

private:
	static DragonLeds *m_instance;
	frc::AddressableLED *m_addressibleLeds;
	int m_rainbowFirstPixelHue = 0;
	int m_numberofDiagnosticLEDs = 8;

	DragonLeds();
};
