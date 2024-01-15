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

// C++ Includes
#include <vector>

// FRC Includes
#include <networktables/GenericEntry.h>

#include <AdjustableItem.h>

class AdjustableItemMgr
{
public:
    static AdjustableItemMgr *GetInstance();

    /// @brief  add item to vector in state mgr
    /// @param item - each constructed AdjustableItem
    void RegisterAdjustableItem(
        AdjustableItem *item);

    /// @brief Listens for button presses on network table
    void ListenForUpdates();

    /// @brief Check for differences in all adjustable items from parsed value and network table value
    /// @return all adjustable items with changes
    std::vector<AdjustableItem *> CheckForDifferences();

private:
    AdjustableItemMgr();
    ~AdjustableItemMgr() = default;

    /// @brief Will populate the network tables with keys for every adjustable value in an item
    void PopulateNetworkTables();

    std::vector<AdjustableItem *> m_adjustableItems;

    nt::GenericEntry *m_enableButton;
    nt::GenericEntry *m_submitButton;
    nt::GenericEntry *m_resetButton;
    nt::GenericEntry *m_getDiffsButton;

    bool m_enabled = false;
    static AdjustableItemMgr *m_instance;
};