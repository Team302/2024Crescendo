
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
#include <string>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/PowerDistribution.h>
#include <frc/motorcontrol/MotorController.h>

// Team 302 includes
#include "hw/DistanceAngleCalcStruc.h"
#include "hw/interfaces/IDragonMotorController.h"
#include "hw/DragonTalonFX.h"
#include "hw/factories/PDPFactory.h"
#include "hw/factories/DragonControlToCTREV6AdapterFactory.h"
#include "configs/RobotElementNames.h"
#include "utils/logging/Logger.h"
#include "utils/ConversionUtils.h"
#include "hw/ctreadapters/v6/DragonControlToCTREV6Adapter.h"

// Third Party Includes
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"

using namespace frc;
using ctre::phoenix6::configs::CurrentLimitsConfigs;

using ctre::phoenix6::configs::CANcoderConfiguration;
using ctre::phoenix6::configs::ClosedLoopRampsConfigs;
using ctre::phoenix6::configs::HardwareLimitSwitchConfigs;
using ctre::phoenix6::configs::MotorOutputConfigs;
using ctre::phoenix6::configs::OpenLoopRampsConfigs;
using ctre::phoenix6::configs::Slot0Configs;
using ctre::phoenix6::configs::Slot1Configs;
using ctre::phoenix6::configs::Slot2Configs;
using ctre::phoenix6::configs::VoltageConfigs;
using ctre::phoenix6::controls::ControlRequest;
using ctre::phoenix6::controls::Follower;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;
using ctre::phoenix6::signals::FeedbackSensorSourceValue;
using ctre::phoenix6::signals::ForwardLimitSourceValue;
using ctre::phoenix6::signals::ForwardLimitTypeValue;
using ctre::phoenix6::signals::ForwardLimitValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix6::signals::ReverseLimitSourceValue;
using ctre::phoenix6::signals::ReverseLimitTypeValue;
using ctre::phoenix6::signals::ReverseLimitValue;
using ctre::phoenix6::signals::SensorDirectionValue;

using ctre::phoenix6::configs::TalonFXConfiguration;
using ctre::phoenix6::hardware::TalonFX;
using std::shared_ptr;
using std::string;
using std::to_string;

DragonTalonFX::DragonTalonFX(string networkTableName,
							 RobotElementNames::MOTOR_CONTROLLER_USAGE deviceType,
							 int deviceID,
							 const DistanceAngleCalcStruc &calcStruc,
							 MOTOR_TYPE motorType,
							 string canBusName) : m_networkTableName(networkTableName),
												  m_type(deviceType),
												  m_talon(TalonFX(deviceID, canBusName)),
												  m_calcStruc(calcStruc),
												  m_motorType(motorType),
												  m_inverted(false)
{
	ResetToDefaults();
	auto factory = DragonControlToCTREV6AdapterFactory::GetFactory();
	for (auto i = 0; i < 4; ++i)
	{
		m_controller[i] = factory->CreateAdapter(networkTableName,
												 i,
												 ControlData(),
												 m_calcStruc,
												 m_talon);

		m_controller[i]->InitializeDefaults();
	}
}

void DragonTalonFX::ConfigHWLimitSW(bool enableForward,
									int remoteForwardSensorID,
									bool forwardResetPosition,
									double forwardPosition,
									ForwardLimitSourceValue forwardType,
									ForwardLimitTypeValue forwardOpenClose,
									bool enableReverse,
									int remoteReverseSensorID,
									bool reverseResetPosition,
									double reversePosition,
									ReverseLimitSourceValue revType,
									ReverseLimitTypeValue revOpenClose)
{
	HardwareLimitSwitchConfigs hwswitch{};
	hwswitch.ForwardLimitEnable = enableForward;
	hwswitch.ForwardLimitRemoteSensorID = remoteForwardSensorID;
	hwswitch.ForwardLimitAutosetPositionEnable = forwardResetPosition;
	hwswitch.ForwardLimitAutosetPositionValue = forwardPosition;
	hwswitch.ForwardLimitSource = forwardType;
	hwswitch.ForwardLimitType = forwardOpenClose;

	hwswitch.ReverseLimitEnable = enableReverse;
	hwswitch.ReverseLimitRemoteSensorID = remoteReverseSensorID;
	hwswitch.ReverseLimitAutosetPositionEnable = reverseResetPosition;
	hwswitch.ReverseLimitAutosetPositionValue = reversePosition;
	hwswitch.ReverseLimitSource = revType;
	hwswitch.ReverseLimitType = revOpenClose;
	m_talon.GetConfigurator().Apply(hwswitch);
}

void DragonTalonFX::ConfigMotorSettings(InvertedValue inverted,
										NeutralModeValue mode,
										double deadbandPercent,
										double peakForwardDutyCycle,
										double peakReverseDutyCycle)
{
	m_inverted = inverted == InvertedValue::Clockwise_Positive;
	MotorOutputConfigs motorconfig{};
	motorconfig.Inverted = inverted;
	motorconfig.NeutralMode = mode;
	motorconfig.PeakForwardDutyCycle = peakForwardDutyCycle;
	motorconfig.PeakReverseDutyCycle = peakReverseDutyCycle;
	motorconfig.DutyCycleNeutralDeadband = deadbandPercent;
	m_talon.GetConfigurator().Apply(motorconfig);
}
void DragonTalonFX::SetPIDConstants(int slot, double p, double i, double d, double f)
{
	ControlData controlInfo{ControlModes::CONTROL_TYPE::VOLTAGE,
							ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
							std::string("NewController"),
							p, i, d, f,
							ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE,
							0.0, 1.0, 1.0, 1.0, 0.0, true};

	auto factory = DragonControlToCTREV6AdapterFactory::GetFactory();
	delete m_controller[slot];
	m_controller[slot] = factory->CreateAdapter(m_networkTableName, slot, controlInfo, m_calcStruc, m_talon);
}
double DragonTalonFX::GetRotations()
{
	auto &possig = m_talon.GetPosition();
	possig.Refresh();

	auto &velsig = m_talon.GetVelocity();
	auto turnsPerSec = velsig.GetValue();

	auto rotations = possig.GetValue() + turnsPerSec * possig.GetTimestamp().GetLatency();
	return rotations.to<double>();
}

double DragonTalonFX::GetRPS()
{
	auto &velsig = m_talon.GetVelocity();
	auto turnsPerSec = velsig.GetValue();
	return turnsPerSec.to<double>();
}

double DragonTalonFX::GetCurrent()
{
	return m_talon.GetSupplyCurrent().GetValue().to<double>();
}

void DragonTalonFX::Set(double value)
{
	m_controller[0]->Set(value);
}
void DragonTalonFX::Set(ControlRequest &control)
{
	m_talon.SetControl(control);
}

void DragonTalonFX::SetRotationOffset(double rotations)
{
	//	double newRotations = -rotations + DragonTalonFX::GetRotations();
	//	m_tickOffset += (int) (newRotations * m_calcStruc.countsPerRev / m_calcStruc.gearRatio);
}

void DragonTalonFX::SetVoltageRamping(double ramping, double rampingClosedLoop)
{
	ClosedLoopRampsConfigs configs{};
	m_talon.GetConfigurator().Refresh(configs);
	configs.VoltageClosedLoopRampPeriod = rampingClosedLoop;
	m_talon.GetConfigurator().Apply(configs);

	if (rampingClosedLoop > 0.0)
	{
		OpenLoopRampsConfigs oconfigs{};
		m_talon.GetConfigurator().Refresh(oconfigs);
		oconfigs.VoltageOpenLoopRampPeriod = ramping;
		m_talon.GetConfigurator().Apply(oconfigs);
	}
}

void DragonTalonFX::EnableBrakeMode(bool enabled)
{
	MotorOutputConfigs motorconfig{};
	m_talon.GetConfigurator().Refresh(motorconfig);
	if (enabled)
	{
		motorconfig.NeutralMode = NeutralModeValue::Brake;
	}
	else
	{
		motorconfig.NeutralMode = NeutralModeValue::Coast;
	}
	m_talon.GetConfigurator().Apply(motorconfig);
}

void DragonTalonFX::ResetToDefaults()
{
	m_talon.GetConfigurator().Apply(TalonFXConfiguration{}); // reset to factory default
	EnableBrakeMode(true);

	auto invVal = m_inverted ? InvertedValue::Clockwise_Positive : InvertedValue::CounterClockwise_Positive;
	ConfigMotorSettings(invVal, NeutralModeValue::Brake, 0.01, 1.0, -1.0);
}

void DragonTalonFX::Invert(bool inverted)
{
	m_inverted = inverted;
	m_talon.SetInverted(inverted);
}

void DragonTalonFX::SetSensorInverted(bool inverted)
{
}

RobotElementNames::MOTOR_CONTROLLER_USAGE DragonTalonFX::GetType() const
{
	return m_type;
}

int DragonTalonFX::GetID() const
{
	return m_talon.GetDeviceID();
}

void DragonTalonFX::SetCurrentLimits(bool enableStatorCurrentLimit,
									 units::current::ampere_t statorCurrentLimit,
									 bool enableSupplyCurrentLimit,
									 units::current::ampere_t supplyCurrentLimit,
									 units::current::ampere_t supplyCurrentThreshold,
									 units::time::second_t supplyTimeThreshold)
{
	CurrentLimitsConfigs currconfigs{};
	currconfigs.StatorCurrentLimit = statorCurrentLimit.to<double>();
	currconfigs.StatorCurrentLimitEnable = enableSupplyCurrentLimit;
	currconfigs.SupplyCurrentLimit = supplyCurrentLimit.to<double>();
	currconfigs.SupplyCurrentLimitEnable = enableSupplyCurrentLimit;
	m_talon.GetConfigurator().Apply(currconfigs);
}

void DragonTalonFX::EnableCurrentLimiting(bool enabled)
{
	CurrentLimitsConfigs configs{};
	m_talon.GetConfigurator().Refresh(configs);
	configs.StatorCurrentLimitEnable = enabled;
	configs.SupplyCurrentLimitEnable = enabled;
	m_talon.GetConfigurator().Apply(configs);
}

void DragonTalonFX::SetAsFollowerMotor(int masterCANID // <I> - master motor
)
{
	m_talon.SetControl(Follower(masterCANID, false));
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] int             slot - hardware slot to use
/// @param [in] ControlData*    pid - the control constants
/// @return void
void DragonTalonFX::SetControlConstants(int slot, const ControlData &controlInfo)
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonTalonFX::SetControlConstants"), string("P"), controlInfo.GetP());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonTalonFX::SetControlConstants"), string("I"), controlInfo.GetI());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonTalonFX::SetControlConstants"), string("D"), controlInfo.GetD());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonTalonFX::SetControlConstants"), string("F"), controlInfo.GetF());

	auto factory = DragonControlToCTREV6AdapterFactory::GetFactory();
	delete m_controller[slot];
	m_controller[slot] = factory->CreateAdapter(m_networkTableName, slot, controlInfo, m_calcStruc, m_talon);
}

void DragonTalonFX::SetRemoteSensor(int canID,
									ctre::phoenix::motorcontrol::RemoteSensorSource deviceType)
{
	TalonFXConfiguration configs{};
	configs.Feedback.FeedbackRemoteSensorID = canID;
	configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::FusedCANcoder;

	m_talon.GetConfigurator().Apply(configs);
}
void DragonTalonFX::FuseCancoder(DragonCanCoder &cancoder,
								 double sensorToMechanismRatio,
								 double rotorToSensorRatio)
{
	// update cancoder definition
	cancoder.SetRange(AbsoluteSensorRangeValue::Signed_PlusMinusHalf);
	cancoder.SetDirection(SensorDirectionValue::CounterClockwise_Positive);

	TalonFXConfiguration configs{};
	m_talon.GetConfigurator().Refresh(configs);
	configs.Feedback.FeedbackRemoteSensorID = cancoder.GetCanId();
	configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::FusedCANcoder;
	configs.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
	configs.Feedback.RotorToSensorRatio = rotorToSensorRatio;

	m_talon.GetConfigurator().Apply(configs);
}

void DragonTalonFX::SetDiameter(double diameter)
{
	m_calcStruc.diameter = diameter;
}

void DragonTalonFX::SetVoltage(units::volt_t output)
{
	m_talon.SetVoltage(output);
}

bool DragonTalonFX::IsForwardLimitSwitchClosed()
{
	auto signal = m_talon.GetForwardLimit();
	return signal.GetValue() == ForwardLimitValue::ClosedToGround;
}

bool DragonTalonFX::IsReverseLimitSwitchClosed()
{
	auto signal = m_talon.GetReverseLimit();
	return signal.GetValue() == ReverseLimitValue::ClosedToGround;
}

void DragonTalonFX::EnableDisableLimitSwitches(bool enable)
{
	HardwareLimitSwitchConfigs hwswitch{};
	m_talon.GetConfigurator().Refresh(hwswitch);
	hwswitch.ForwardLimitEnable = enable;
	m_talon.GetConfigurator().Apply(hwswitch);
}

void DragonTalonFX::EnableVoltageCompensation(double fullvoltage)
{
	// m_talon.ConfigVoltageCompSaturation(fullvoltage);
	// m_talon.EnableVoltageCompensation(true);
}

void DragonTalonFX::SetSelectedSensorPosition(double initialPosition)
{
	m_talon.SetPosition(units::angle::degree_t(initialPosition));
}

double DragonTalonFX::GetCountsPerInch() const
{
	return m_calcStruc.countsPerInch;
}
double DragonTalonFX::GetCountsPerDegree() const
{
	return m_calcStruc.countsPerDegree;
}

/**
ControlModes::CONTROL_TYPE DragonTalonFX::GetControlMode() const
{
	return m_controlMode;
}
**/

double DragonTalonFX::GetCounts()
{
	return GetRotations() * 2048;
}

IDragonMotorController::MOTOR_TYPE DragonTalonFX::GetMotorType() const
{
	return MOTOR_TYPE::FALCON500;
}

void DragonTalonFX::MonitorCurrent()
{
}