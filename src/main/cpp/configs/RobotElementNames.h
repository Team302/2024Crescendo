// clang-format off
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Thursday, January 25, 2024 8:27:52 PM

#pragma once

class RobotElementNames
{

public:
	enum MOTOR_CONTROLLER_USAGE
	{
		UNKNOWN_MOTOR_CONTROLLER = -1,
		NOTE_MANAGER_FRONT_INTAKE,
		NOTE_MANAGER_BACK_INTAKE,
		NOTE_MANAGER_TRANSFER,
		NOTE_MANAGER_PLACER,
		NOTE_MANAGER_ELEVATOR,
		NOTE_MANAGER_FEEDER,
		NOTE_MANAGER_LAUNCHER_TOP,
		NOTE_MANAGER_LAUNCHER_BOTTOM,
		NOTE_MANAGER_LAUNCHER_ANGLE,
		MAX_MOTOR_CONTROLLER
	};

	enum FEEDBACK_SENSOR_CONFIG_BASE_USAGE
	{
		UNKNOWN_FEEDBACK_SENSOR_CONFIG_BASE = -1,
		MAX_FEEDBACK_SENSOR_CONFIG_BASE
	};

	enum CANCODER_INSTANCE_USAGE
	{
		UNKNOWN_CANCODER_INSTANCE = -1,
		MAX_CANCODER_INSTANCE
	};

	enum PID_USAGE
	{
		UNKNOWN_PID = -1,
		MAX_PID
	};

	enum PDP_USAGE
	{
		UNKNOWN_PDP = -1,
		PDP,
		MAX_PDP
	};

	enum PIGEON_USAGE
	{
		UNKNOWN_PIGEON = -1,
		PIGEON_ROBOT_CENTER,
		MAX_PIGEON
	};

	enum PCM_USAGE
	{
		UNKNOWN_PCM = -1,
		MAX_PCM
	};

	enum ANALOG_INPUT_USAGE
	{
		UNKNOWN_ANALOG_INPUT = -1,
		MAX_ANALOG_INPUT
	};

	enum LIMELIGHT_USAGE
	{
		UNKNOWN_LIMELIGHT = -1,
		MAX_LIMELIGHT
	};

	enum CHASSIS_USAGE
	{
		UNKNOWN_CHASSIS = -1,
		MAX_CHASSIS
	};

	enum DIGITAL_INPUT_USAGE
	{
		UNKNOWN_DIGITAL_INPUT = -1,
		NOTE_MANAGER_FRONT_INTAKE_SENSOR,
		NOTE_MANAGER_BACK_INTAKE_SENSOR,
		NOTE_MANAGER_FEEDER_SENSOR,
		NOTE_MANAGER_LAUNCHER_SENSOR,
		NOTE_MANAGER_PLACER_IN_SENSOR,
		NOTE_MANAGER_PLACER_MID_SENSOR,
		NOTE_MANAGER_PLACER_OUT_SENSOR,
		MAX_DIGITAL_INPUT
	};

	enum SWERVE_MODULE_USAGE
	{
		UNKNOWN_SWERVE_MODULE = -1,
		MAX_SWERVE_MODULE
	};

	enum CANCODER_USAGE
	{
		UNKNOWN_CANCODER = -1,
		NOTE_MANAGER_LAUNCHER_ANGLE_SENSOR,
		MAX_CANCODER
	};

	enum SOLENOID_USAGE
	{
		UNKNOWN_SOLENOID = -1,
		MAX_SOLENOID
	};

	enum SERVO_USAGE
	{
		UNKNOWN_SERVO = -1,
		MAX_SERVO
	};

	enum COLOR_SENSOR_USAGE
	{
		UNKNOWN_COLOR_SENSOR = -1,
		MAX_COLOR_SENSOR
	};

	enum CAMERA_USAGE
	{
		UNKNOWN_CAMERA = -1,
		MAX_CAMERA
	};

	enum ROBORIO_USAGE
	{
		UNKNOWN_ROBORIO = -1,
		MAX_ROBORIO
	};

	enum LED_USAGE
	{
		UNKNOWN_LED = -1,
		MAX_LED
	};

	enum TALONTACH_USAGE
	{
		UNKNOWN_TALONTACH = -1,
		MAX_TALONTACH
	};

	enum MOTOR_CONTROL_DATA_USAGE
	{
		UNKNOWN_MOTOR_CONTROL_DATA = -1,
		NOTE_MANAGER_PERCENT_OUTPUT,
		NOTE_MANAGER_POSITION_INCH,
		NOTE_MANAGER_VELOCITY_RPS,
		NOTE_MANAGER_POS_DEGREE_ABS,
		MAX_MOTOR_CONTROL_DATA
	};

	enum MOTOR_CONTROL_DATA_LINK_USAGE
	{
		UNKNOWN_MOTOR_CONTROL_DATA_LINK = -1,
		MAX_MOTOR_CONTROL_DATA_LINK
	};

	enum MOTOR_TARGET_USAGE
	{
		UNKNOWN_MOTOR_TARGET = -1,
		MAX_MOTOR_TARGET
	};

	enum STATE_USAGE
	{
		UNKNOWN_STATE = -1,
		NOTE_MANAGER_OFF,
		NOTE_MANAGER_READY,
		NOTE_MANAGER_FEEDER_INTAKE,
		NOTE_MANAGER_EXPEL,
		NOTE_MANAGER_PLACER_INTAKE,
		NOTE_MANAGER_HOLD_FEEDER_FRONT,
		NOTE_MANAGER_HOLD_FEEDER_BACK,
		NOTE_MANAGER_INTAKE_TO_FEEDER,
		NOTE_MANAGER_LAUNCHER_TO_PLACER_FRONT,
		NOTE_MANAGER_LAUNCHER_TO_PLACER_BACK,
		NOTE_MANAGER_HOLD_FEEDER,
		NOTE_MANAGER_READY_AUTO_LAUNCH,
		NOTE_MANAGER_READY_MANUAL_LAUNCH,
		NOTE_MANAGER_PASS,
		NOTE_MANAGER_AUTO_LAUNCH,
		NOTE_MANAGER_MANUAL_LAUNCH,
		NOTE_MANAGER_READY_ODOMETRY_LAUNCH,
		NOTE_MANAGER_AUTO_LAUNCH_ODOMETRY,
		NOTE_MANAGER_HOLD_PLACER_FRONT,
		NOTE_MANAGER_HOLD_PLACER_BACK,
		NOTE_MANAGER_INTAKE_TO_PLACER,
		NOTE_MANAGER_PREPARE_PLACE_AMP,
		NOTE_MANAGER_PREPARE_PLACE_TRAP,
		NOTE_MANAGER_PLACE_AMP,
		NOTE_MANAGER_PLACE_TRAP,
		NOTE_MANAGER_PLACER_TO_LAUNCHER_FRONT,
		NOTE_MANAGER_PLACER_TO_LAUNCHER_BACK,
		NOTE_MANAGER_BACKUP_MANUAL_LAUNCH,
		NOTE_MANAGER_BACKUP_MANUAL_PLACE,
		MAX_STATE
	};

private:
	RobotElementNames() = delete;
	~RobotElementNames() = delete;
};
