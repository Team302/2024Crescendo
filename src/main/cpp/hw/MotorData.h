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

/// --------------------------------------------------------------------------------------------
/// @class MotorData
/// @brief
///        Creating a motor data class to get the stall current, free current, etc. of a motor.
/// --------------------------------------------------------------------------------------------
#pragma once
// Third Party Includes

// 302 includes
#include "hw/interfaces/IDragonMotorController.h"

class MotorData
{
public:
    int getStallCurrent(IDragonMotorController::MOTOR_TYPE motorType) const;
    double getFreeCurrent(IDragonMotorController::MOTOR_TYPE motorType) const;
    int getFreeSpeed(IDragonMotorController::MOTOR_TYPE motorType) const;
    int getMaximumPower(IDragonMotorController::MOTOR_TYPE motorType) const;
    double getStallTorque(IDragonMotorController::MOTOR_TYPE motorType) const;
    bool checkIfStall(std::shared_ptr<IDragonMotorController> motor);
    static MotorData *GetInstance();

private:
    MotorData() = default;
    ~MotorData() = default;

    const int stallCurrentValues[IDragonMotorController::MOTOR_TYPE::MAX_MOTOR_TYPES] =
        {
            257, // Falcon 500
            166, // NEO Motor
            111, // NEO 550 Motor
            131, // CIM Motor
            89,  // Mini CIM Motor
            53,  // BAG Motor
            134, // 775pro Motor
            71,  // AndyMark 9015
            10,  // AndyMark NeveRest
            18,  // AndyMark RS775-125
            122, // AndyMark Redline A
            11,  // REV Robotgics HD Hex Motor
            97,  // BaneBots RS-775 18V
            84,  // BaneBots RS-550
            11,  // Modern Robotics 12VDC Motor
            21,  // Johnson Electric Gear Motor
            9,   // TETRIX MAX TorqueNADO Motor
            366, // Kraken x60
            211, // Neo Vortex
            0    // No motor
    };

    const double freeCurrentValues[IDragonMotorController::MOTOR_TYPE::MAX_MOTOR_TYPES] =
        {
            1.5, // Falcon 500
            1.3, // NEO Motor
            1.1, // NEO 550 Motor
            2.7, // CIM Motor
            3,   // Mini CIM Motor
            1.8, // BAG Motor
            0.7, // 775pro Motor
            3.7, // AndyMark 9015
            0.4, // AndyMark NeveRest
            1.6, // AndyMark RS775-125
            2.6, // AndyMark Redline A
            0.3, // REV Robotgics HD Hex Motor
            2.7, // BaneBots RS-775 18V
            0.4, // BaneBots RS-550
            0.3, // Modern Robotics 12VDC Motor
            0.9, // Johnson Electric Gear Motor
            0.2, // TETRIX MAX TorqueNADO Motor
            2,   // Kraken x 60
            3.6, // Neo Vortex
            0    // No motor
    };

    const int maximumPowerValues[IDragonMotorController::MOTOR_TYPE::MAX_MOTOR_TYPES] =
        {
            783,  // Falcon 500
            516,  // NEO Motor
            332,  // NEO 550 Motor
            337,  // CIM Motor
            215,  // Mini CIM Motor
            149,  // BAG Motor
            347,  // 775pro Motor
            134,  // AndyMark 9015
            25,   // AndyMark NeveRest
            43,   // AndyMark RS775-125
            327,  // AndyMark Redline A
            28,   // REV Robotgics HD Hex Motor
            246,  // BaneBots RS-775 18V
            190,  // BaneBots RS-550
            29,   // Modern Robotics 12VDC Motor
            45,   // Johnson Electric Gear Motor
            26,   // TETRIX MAX TorqueNADO Motor
            1108, // Kraken x 60
            640,  // Neo Vortex
            0     // No motor
    };

    const double stallTorqueValues[IDragonMotorController::MOTOR_TYPE::MAX_MOTOR_TYPES] =
        {
            4.69,  // Falcon 500
            3.36,  // NEO Motor
            1.08,  // NEO 550 Motor
            2.41,  // CIM Motor
            1.41,  // Mini CIM Motor
            0.43,  // BAG Motor
            0.71,  // 775pro Motor
            0.36,  // AndyMark 9015
            0.17,  // AndyMark NeveRest
            0.28,  // AndyMark RS775-125
            0.64,  // AndyMark Redline A
            0.182, // REV Robotgics HD Hex Motor
            0.72,  // BaneBots RS-775 18V
            0.38,  // BaneBots RS-550
            0.19,  // Modern Robotics 12VDC Motor
            4.09,  // Johnson Electric Gear Motor
            0.17,  // TETRIX MAX TorqueNADO Motor
            7.09,  // Kraken x 60
            3.6,   // Neo Vortex
            0      // No motor
    };

    const int freeSpeedValues[IDragonMotorController::MOTOR_TYPE::MAX_MOTOR_TYPES] =
        {
            6380,  // Falcon 500
            5880,  // NEO Motor
            11710, // NEO 550 Motor
            5330,  // CIM Motor
            5840,  // Mini CIM Motor
            13180, // BAG Motor
            18730, // 775pro Motor
            14270, // AndyMark 9015
            5480,  // AndyMark NeveRest
            5800,  // AndyMark RS775-125
            19500, // AndyMark Redline A
            5960,  // REV Robotgics HD Hex Motor
            13050, // BaneBots RS-775 18V
            19000, // BaneBots RS-550
            5900,  // Modern Robotics 12VDC Motor
            420,   // Johnson Electric Gear Motor
            5920,  // TETRIX MAX TorqueNADO Motor
            6000,  // Kraken x 60
            6784,  // Neo Vortex
            0      // No motor
    };

    static MotorData *m_instance;
};