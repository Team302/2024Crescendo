
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
#include <memory>
#include <numbers>
#include <string>

// FRC includes
#include "frc/Filesystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "chassis/SwerveChassis.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "hw/DragonCanCoder.h"
#include "mechanisms/controllers/ControlData.h"
#include "utils/AngleUtils.h"
#include "utils/logging/Logger.h"

// Third Party Includes
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "pugixml/pugixml.hpp"

using namespace std;
using namespace frc;

using std::string;

using ctre::phoenix6::configs::CANcoderConfiguration;
using ctre::phoenix6::configs::CANcoderConfigurator;
using ctre::phoenix6::configs::CurrentLimitsConfigs;
using ctre::phoenix6::configs::MotorOutputConfigs;
using ctre::phoenix6::configs::Slot0Configs;
using ctre::phoenix6::configs::TalonFXConfiguration;
using ctre::phoenix6::configs::VoltageConfigs;
using ctre::phoenix6::controls::DutyCycleOut;
using ctre::phoenix6::controls::PositionVoltage;
using ctre::phoenix6::controls::VelocityTorqueCurrentFOC;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;
using ctre::phoenix6::signals::FeedbackSensorSourceValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix6::signals::SensorDirectionValue;

//==================================================================================
SwerveModule::SwerveModule(string canbusname,
                           SwerveModuleConstants::ModuleID id,
                           SwerveModuleConstants::ModuleType type,
                           int driveMotorID,
                           bool driveInverted,
                           int turnMotorID,
                           bool turnInverted,
                           int canCoderID,
                           bool canCoderInverted,
                           double angleOffset,
                           string configfilename,
                           string networkTableName) : LoggableItem(),
                                                      m_moduleID(id),
                                                      m_driveTalon(new TalonFX(driveMotorID, canbusname)),
                                                      m_turnTalon(new TalonFX(turnMotorID, canbusname)),
                                                      m_turnCancoder(new CANcoder(canCoderID, canbusname)),
                                                      m_activeState(),
                                                      m_networkTableName(networkTableName)
{
    ReadConstants(configfilename);

    Rotation2d ang{units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;

    auto attrs = SwerveModuleConstants::GetSwerveModuleAttrs(type);
    m_wheelDiameter = attrs.wheelDiameter;
    m_maxSpeed = attrs.maxSpeed;

    InitDriveMotor(driveInverted);
    InitTurnMotorEncoder(turnInverted, canCoderInverted, angleOffset, attrs);
}

//==================================================================================
/// @brief Get the encoder values
/// @returns double - the integrated sensor position
double SwerveModule::GetEncoderValues()
{
    return m_driveTalon->GetPosition().GetValueAsDouble() * 2048;
}

//==================================================================================
/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    SetTurnAngle(units::degree_t(0));
}

//==================================================================================
/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * numbers::pi);
    units::velocity::meters_per_second_t mps = units::velocity::meters_per_second_t(0.0);
    mps = units::velocity::meters_per_second_t(mpr.to<double>() * m_driveTalon->GetVelocity().GetValueAsDouble());

    // Get the Module Current Rotation Angle
    units::angle::degree_t ang = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d angle = Rotation2d(ang);

    // Create the state and return it
    SwerveModuleState state{mps, angle};
    return state;
}

//==================================================================================
/// @brief Get the current position of the swerve module (distance and rotation)
/// @return frc::SwerveModulePosition - current position
frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    double rotations = 0.0;
    rotations = m_driveTalon->GetPosition().GetValueAsDouble() / 6.12;
    units::angle::degree_t angle = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);

    return {rotations * m_wheelDiameter * numbers::pi, // distance travled by drive motor
            currAngle};                                // angle of the swerve module from sensor
}

//==================================================================================
/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState(const SwerveModuleState &targetState)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    units::angle::degree_t angle = m_turnCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);
    auto optimizedState = SwerveModuleState::Optimize(targetState, currAngle);

    // Set Turn Target
    SetTurnAngle(optimizedState.angle.Degrees());

    // Set Drive Target
    SetDriveSpeed(optimizedState.speed);
}

//==================================================================================
/// @brief Run the swerve module at the same speed and angle
/// @returns void
void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_activeState.speed);
}

//==================================================================================
/// @brief run the drive motor at a specified speed
/// @param [in] speed to drive the drive wheel as
/// @returns void
void SwerveModule::SetDriveSpeed(units::velocity::meters_per_second_t speed)
{
    // Run Open Loop
    m_activeState.speed = (abs(speed.to<double>() / m_maxSpeed.to<double>()) < 0.05) ? 0_mps : speed;
    // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
    auto driveTarget = m_activeState.speed.to<double>() / (units::length::meter_t(m_wheelDiameter).to<double>() * numbers::pi);
    auto percent = driveTarget / m_maxSpeed.to<double>();
    DutyCycleOut out{percent};
    m_driveTalon->SetControl(out);
}

//==================================================================================
/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetTurnAngle(units::angle::degree_t targetAngle)
{
    PositionVoltage voltagePosition{0_tr, 0_tps, true, 0_V, 0, false};
    m_activeState.angle = targetAngle;
    m_turnTalon->SetControl(voltagePosition.WithPosition(targetAngle));
}

//==================================================================================
/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    SetDriveSpeed(0_mps);
}

//==================================================================================
void SwerveModule::LogInformation()
{
    string ntAngleName;
    string ntMotorPositionName;
    string ntRotorPositionName;
    if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_BACK)
    {
        ntAngleName += string("leftback Angle");
        ntMotorPositionName += string("leftback turns");
        ntRotorPositionName += string("leftback rotor");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_FRONT)
    {
        ntAngleName += string("leftfront Angle");
        ntMotorPositionName += string("leftfront turns");
        ntRotorPositionName += string("leftfront rotor");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::RIGHT_BACK)
    {
        ntAngleName += string("rightback Angle");
        ntMotorPositionName += string("rightback turns");
        ntRotorPositionName += string("rightback rotor");
    }
    else
    {
        ntAngleName += string("rightfront Angle");
        ntMotorPositionName += string("rightfront turns");
        ntRotorPositionName += string("rightfront rotor");
    }
    auto turns = m_turnTalon->GetPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, ntMotorPositionName, turns);

    auto rotor = m_turnTalon->GetRotorPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, ntRotorPositionName, rotor);
}

//==================================================================================
void SwerveModule::InitDriveMotor(bool driveInverted)
{
    if (m_driveTalon != nullptr)
    {
        m_driveTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        MotorOutputConfigs motorconfig{};
        motorconfig.Inverted = driveInverted ? InvertedValue::Clockwise_Positive : InvertedValue::CounterClockwise_Positive;
        motorconfig.NeutralMode = NeutralModeValue::Brake;
        motorconfig.PeakForwardDutyCycle = 1.0;
        motorconfig.PeakReverseDutyCycle = -1.0;
        motorconfig.DutyCycleNeutralDeadband = 0.0;
        m_driveTalon->GetConfigurator().Apply(motorconfig);

        CurrentLimitsConfigs currconfig{};
        // limit stator current to 25 amps
        currconfig.StatorCurrentLimit = 40.0;
        currconfig.StatorCurrentLimitEnable = true;

        currconfig.SupplyCurrentLimit = 40.0;
        currconfig.SupplyCurrentLimitEnable = true;
        currconfig.SupplyCurrentThreshold = 30.0;
        currconfig.SupplyTimeThreshold = 0.25;
        m_driveTalon->GetConfigurator().Apply(currconfig);

        VoltageConfigs voltconfig{};
        voltconfig.PeakForwardVoltage = 11.0;
        voltconfig.PeakReverseVoltage = -11.0;
        m_driveTalon->GetConfigurator().Apply(voltconfig);

        m_driveTalon->SetInverted(driveInverted);
    }
}

//==================================================================================
void SwerveModule::InitTurnMotorEncoder(bool turnInverted,
                                        bool canCoderInverted,
                                        double angleOffset,
                                        const SwerveModuleAttributes &attrs)
{
    if (m_turnTalon != nullptr && m_turnCancoder != nullptr)
    {
        m_turnTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        TalonFXConfiguration fxconfigs{};
        m_turnTalon->GetConfigurator().Refresh(fxconfigs);

        fxconfigs.MotorOutput.Inverted = turnInverted ? InvertedValue::Clockwise_Positive : InvertedValue::Clockwise_Positive;
        fxconfigs.MotorOutput.NeutralMode = NeutralModeValue::Brake;
        fxconfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
        fxconfigs.MotorOutput.PeakReverseDutyCycle = -1.0;
        fxconfigs.MotorOutput.DutyCycleNeutralDeadband = 0.0;

        fxconfigs.Voltage.PeakForwardVoltage = 10.0;
        fxconfigs.Voltage.PeakReverseVoltage = -10.0;

        fxconfigs.Slot0.kP = m_turnKp;
        fxconfigs.Slot0.kI = m_turnKi;
        fxconfigs.Slot0.kD = m_turnKd;

        fxconfigs.ClosedLoopGeneral.ContinuousWrap = true;

        fxconfigs.Feedback.FeedbackRemoteSensorID = m_turnCancoder->GetDeviceID();
        // fxconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::SyncCANcoder;
        fxconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RemoteCANcoder;
        fxconfigs.Feedback.SensorToMechanismRatio = attrs.sensorToMechanismRatio;
        fxconfigs.Feedback.RotorToSensorRatio = attrs.rotorToSensorRatio;
        m_turnTalon->GetConfigurator().Apply(fxconfigs);

        m_turnTalon->SetInverted(turnInverted);

        CANcoderConfiguration ccConfigs{};
        m_turnCancoder->GetConfigurator().Apply(ccConfigs); // Apply Factory Defaults

        ccConfigs.MagnetSensor.MagnetOffset = angleOffset;
        ccConfigs.MagnetSensor.SensorDirection = canCoderInverted ? SensorDirectionValue::Clockwise_Positive : SensorDirectionValue::CounterClockwise_Positive;
        ccConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

        m_turnCancoder->GetConfigurator().Apply(ccConfigs);
    }
}

//==================================================================================
void SwerveModule::ReadConstants(string configfilename)
{
    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto filename = deployDir + string("/chassis/") + configfilename;
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());

    if (result)
    {
        pugi::xml_node parent = doc.root();
        for (pugi::xml_node swervemod = parent.first_child(); swervemod; swervemod = swervemod.next_sibling())
        {
            for (pugi::xml_node control = swervemod.first_child(); swervemod; swervemod = swervemod.next_sibling())
            {
                for (pugi::xml_attribute attr = control.first_attribute(); attr; attr = attr.next_attribute())
                {
                    if (strcmp(attr.name(), "proportional") == 0)
                    {
                        m_turnKp = attr.as_double();
                    }
                    else if (strcmp(attr.name(), "integral") == 0)
                    {
                        m_turnKi = attr.as_double();
                    }
                    else if (strcmp(attr.name(), "derivative") == 0)
                    {
                        m_turnKd = attr.as_double();
                    }
                }
            }
        }
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("Config File not found"), configfilename);
    }
}
