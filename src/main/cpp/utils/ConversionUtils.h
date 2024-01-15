
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

#include <numbers>

///	 @class ConversionUtils
///  @brief	Collection to convert a number between different units
class ConversionUtils
{
public:
	ConversionUtils() = delete;
	~ConversionUtils() = delete;

	//-------------  Angular Position Conversions

	/// @brief Convert from revolutions to Degrees
	/// @param [in] revolutions
	/// @return double angle in degrees
	inline static double RevolutionsToDegrees(double revolutions) { return revolutions * 360.0; }

	/// @brief Convert from Degrees to revolutions
	/// @param [in] Degrees
	/// @return double number of revolutions
	inline static double DegreesTorevolutions(double degrees) { return degrees / 360.0; }

	/// @brief Convert from degrees to counts
	/// @param [in] degrees
	/// @param [in] counts per revolution
	/// @return double units
	inline static double DegreesToCounts(double degrees, int countsPerRev) { return DegreesTorevolutions(degrees) * (double)countsPerRev; }

	/// @brief Convert from revolutions to counts
	/// @param [in] revolutions
	/// @param [in] counts per revolution
	/// @return double units
	inline static double RevolutionsToCounts(double revs, int countsPerRev) { return revs * (double)countsPerRev; }

	/// @brief Convert from counts to degrees
	/// @param [in] degrees
	/// @param [in] counts per revolution
	/// @return double units
	inline static double CountsToDegrees(int counts, int countsPerRev) { return RevolutionsToDegrees(CountsToRevolutions(counts, countsPerRev)); }

	/// @brief Convert from counts to revolutions
	/// @param [in] revolutions
	/// @param [in] counts per revolution
	/// @return double units
	inline static double CountsToRevolutions(int counts, int countsPerRev) { return ((double)counts / (double)countsPerRev); }

	/// @brief  convert a number in degress to a number in radians
	/// @param degrees
	/// @return value in radians
	inline static double DegreesToRadians(double degrees) { return degrees * std::numbers::pi / 180.0; }

	/// @brief  convert a number in radians to a number in degrees
	/// @param radians
	/// @return value in degrees
	inline static double RadiansToDegress(double radians) { return radians * 180.0 / std::numbers::pi; }

	//-------------  Linear Position Conversions

	/// @brief Convert from revolutions to Inches
	/// @param [in] revolutions
	/// @param [in] diameter in inches
	/// @return double position in inches
	inline static double RevolutionsToInches(double revolutions, double diameter) { return revolutions * std::numbers::pi * diameter; }

	/// @brief Convert from Inches to revolutions
	/// @param [in] Inches
	/// @param [in] diameter in inches
	/// @return double number of revolutions
	inline static double InchesToRevolutions(double inches, double diameter) { return inches / (std::numbers::pi * diameter); }

	/// @brief Convert from counts to Inches
	/// @param [in] counts
	/// @param [in] counts per revolution
	/// @param [in] diameter in inches
	/// @return double position in inches
	inline static double CountsToInches(int counts, int countsPerRev, double diameter) { return RevolutionsToInches(CountsToRevolutions(counts, countsPerRev), diameter); }

	/// @brief Convert from Inches to counts
	/// @param [in] Inches
	/// @param [in] diameter in inches
	/// @return double number of revolutions
	inline static double InchesToCounts(double inches, int countsPerRev, double diameter) { return RevolutionsToCounts(InchesToRevolutions(inches, diameter), countsPerRev); }

	//-------------  Anglular Velocity Conversions

	/// @brief Convert from RPS to Degrees per second
	/// @param [in] revolutions per second
	/// @return double degrees per second
	inline static double RPSToDegreesPerSecond(double rps) { return rps * 360.0; }

	/// @brief Convert from RPS to units/100ms (ctre units)
	/// @param [in] revolutions per second
	/// @param [in] counts per revolution
	/// @return double units / 100 ms
	inline static double RPSToCounts100ms(double rps, int countsPerRev) { return rps * (double)countsPerRev / 10.0; } // rev / sec * counts / rev  * 0.1 sec/100 ms

	/// @brief Convert from Degrees per second to RPS
	/// @param [in] Degrees per second
	/// @return double revolutions per second
	inline static double DegreesPerSecondToRPS(double degreesPerSecond) { return degreesPerSecond / 360.0; }

	/// @brief Convert from Degrees per second to units/100ms (ctre units)
	/// @param [in] Degrees per second
	/// @param [in] counts per revolution
	/// @return double revolutions per second
	inline static double DegreesPerSecondToCounts100ms(double degreesPerSecond, int countsPerRev) { return RPSToCounts100ms((degreesPerSecond / 360.0), countsPerRev); }

	/// @brief Convert from counts per 100ms (ctre units) to revolutions per second
	/// @param [in] counts per 100ms
	/// @param [in] counts per revolution
	/// @return double RPS
	// inline static double CountsPer100msToRPS( double countsPer100ms, int countsPerRev ) { return CountsToRevolutions(countsPer100ms,countsPerRev)*10.0; }
	inline static double CountsPer100msToRPS(double countsPer100ms, int countsPerRev) { return 10 * countsPer100ms / countsPerRev; }

	//-------------  Linear Velocity Conversions

	/// @brief Convert from RPS to inches per second
	/// @param [in] revolutions per second
	/// @param [in] diameter in inches
	/// @return double inches per second
	inline static double RPSToInchesPerSecond(double rps, double diameter) { return rps * std::numbers::pi * diameter; }

	/// @brief Convert from inches per second to RPS
	/// @param [in] inches per second
	/// @param [in] diameter in inches
	/// @return double revolutions per second
	inline static double InchesPerSecondToRPS(double inchesPerSecond, double diameter) { return inchesPerSecond / (std::numbers::pi * diameter); }

	/// @brief Convert from inches per second to units/100ms (ctre units)
	/// @param [in] inches per second
	/// @param [in] counts per revolution
	/// @param [in] diameter in inches
	/// @return double revolutions per second
	inline static double InchesPerSecondToCounts100ms(double inchesPerSecond, int countsPerRev, double diameter) { return InchesToCounts(inchesPerSecond, countsPerRev, diameter) / 0.1; }
};
