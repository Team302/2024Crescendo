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

#include <map>
#include <string>

class RoboRioOrientation
{
public:
    enum ROBORIO_ORIENTATION
    {
        // Z Axis Up
        X_FORWARD_Y_LEFT,
        X_LEFT_Y_BACKWARD,
        X_BACKWARD_Y_RIGHT,
        X_RIGHT_Y_FORWARD,
        // Z Axis Down
        X_FORWARD_Y_RIGHT,
        X_LEFT_Y_FORWARD,
        X_BACKWARD_Y_LEFT,
        X_RIGHT_Y_BACKWARD,
        // Z Axis Backward
        X_UP_Y_LEFT,
        X_LEFT_Y_DOWN,
        X_DOWN_Y_RIGHT,
        X_RIGHT_Y_UP,
        // Z Axis Forward
        X_UP_Y_RIGHT,
        X_LEFT_Y_UP,
        X_DOWN_Y_LEFT,
        X_RIGHT_Y_DOWN
    };

    const std::map<std::string, ROBORIO_ORIENTATION> ROBORIO_ORIENTATION_MAP{
        {std::string("X_FORWARD_Y_LEFT"), X_FORWARD_Y_LEFT},
        {std::string("X_LEFT_Y_BACKWARD"), X_LEFT_Y_BACKWARD},
        {std::string("X_BACKWARD_Y_RIGHT"), X_BACKWARD_Y_RIGHT},
        {std::string("X_RIGHT_Y_FORWARD"), X_RIGHT_Y_FORWARD},
        {std::string("X_FORWARD_Y_RIGHT"), X_FORWARD_Y_RIGHT},
        {std::string("X_LEFT_Y_FORWARD"), X_LEFT_Y_FORWARD},
        {std::string("X_BACKWARD_Y_LEFT"), X_BACKWARD_Y_LEFT},
        {std::string("X_RIGHT_Y_BACKWARD"), X_RIGHT_Y_BACKWARD},
        {std::string("X_UP_Y_LEFT"), X_UP_Y_LEFT},
        {std::string("X_LEFT_Y_DOWN"), X_LEFT_Y_DOWN},
        {std::string("X_DOWN_Y_RIGHT"), X_DOWN_Y_RIGHT},
        {std::string("X_RIGHT_Y_UP"), X_RIGHT_Y_UP},
        {std::string("X_UP_Y_RIGHT"), X_UP_Y_RIGHT},
        {std::string("X_LEFT_Y_UP"), X_LEFT_Y_UP},
        {std::string("X_DOWN_Y_LEFT"), X_DOWN_Y_LEFT},
        {std::string("X_RIGHT_Y_DOWN"), X_RIGHT_Y_DOWN}};

    /// @brief  Find or create the state manmanager
    static RoboRioOrientation *GetInstance();

private:
    RoboRioOrientation() = default;
    ~RoboRioOrientation() = default;

    static RoboRioOrientation *m_instance;
};
