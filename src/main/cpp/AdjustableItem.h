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

// FRC Includes
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableInstance.h>

/// @brief Interface for adjustable/tunable items for testing purposes
class AdjustableItem
{
public:
    AdjustableItem();
    virtual ~AdjustableItem() = default;

    /// @brief Set values based on network table values
    virtual void SetValues() = 0;

    /// @brief Reset network tables and target values to their defaults (parsed from xml)
    virtual void ResetValues() = 0;

    /// @brief Return if there are differences between xml/default values and network table values
    /// @return bool - item has differences
    virtual bool HasDifferences() = 0;

    /// @brief Will log the values that are different from xml
    virtual void ShowDifferences() = 0;

    /// @brief Add adjustable values onto networktable (under the name from xml plus "-Tuner")
    virtual void PopulateNetworkTable() = 0;
};