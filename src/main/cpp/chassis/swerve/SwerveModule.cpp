
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
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "chassis/PoseEstimatorEnum.h"
#include "chassis/swerve/SwerveChassis.h"
#include "chassis/swerve/SwerveModule.h"
#include "hw/DragonCanCoder.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/ControlModes.h"
#include "utils/AngleUtils.h"
#include "utils/logging/Logger.h"

// Third Party Includes
// #include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
// #include "ctre/phoenix/sensors/CANCoder.h"
#include "ctre/phoenix6/CANcoder.hpp"

using namespace std;
using namespace frc;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;

// using namespace ctre::phoenix;
// using namespace ctre::phoenix::motorcontrol::can;
// using namespace ctre::phoenix::sensors;

/// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
/// @param [in] ModuleID                                                type:           Which Swerve Module is it
/// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move
/// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module
/// @param [in] DragonCanCoder*                                 		canCoder:       Sensor for detecting the angle of the wheel
/// @param [in] units::length::inch_t                                   wheelDiameter   Diameter of the wheel
SwerveModule::SwerveModule(ModuleID type,
                           IDragonMotorController *driveMotor,
                           IDragonMotorController *turnMotor,
                           DragonCanCoder *canCoder,
                           const ControlData &controlData,
                           double countsOnTurnEncoderPerDegreesOnAngleSensor,
                           units::length::inch_t wheelDiameter) : m_type(type),
                                                                  m_driveMotor(driveMotor),
                                                                  m_turnMotor(turnMotor),
                                                                  m_turnSensor(canCoder),
                                                                  m_driveVelocityControlData(ControlData()),
                                                                  m_drivePercentControlData(ControlData()),
                                                                  m_turnPositionControlData(controlData),
                                                                  m_wheelDiameter(wheelDiameter),
                                                                  m_nt(),
                                                                  m_activeState(),
                                                                  m_currentPose(),
                                                                  m_currentSpeed(0.0_rpm),
                                                                  m_currentRotations(0.0),
                                                                  m_maxVelocity(1_mps),
                                                                  m_runClosedLoopDrive(false),
                                                                  m_countsOnTurnEncoderPerDegreesOnAngleSensor(countsOnTurnEncoderPerDegreesOnAngleSensor)
{
    turnMotor->SetControlConstants(0, m_turnPositionControlData);

    Rotation2d ang{units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;

    // Set up the Drive Motor
    // TODO replace next block
    /**
    auto motor = m_driveMotor->GetSpeedController();
    auto fx = dynamic_cast<WPI_TalonFX *>(motor);

    // fx->ConfigOpenloopRamp(0.4, 0);
    // fx->ConfigClosedloopRamp(0.4, 0);

    fx->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10);
    fx->ConfigIntegratedSensorInitializationStrategy(BootToZero);
    auto driveMotorSensors = fx->GetSensorCollection();
    driveMotorSensors.SetIntegratedSensorPosition(0, 0);
    **/

    // Set up the Absolute Turn Sensor
    m_turnSensor->SetRange(AbsoluteSensorRangeValue::Signed_PlusMinusHalf);

    // Set up the Turn Motor
    // TODO: Add method to set encoder position to 0 and set it on the turn motor

    switch (GetType())
    {
    case ModuleID::LEFT_FRONT:
        m_nt = string("LeftFrontSwerveModule");
        break;

    case ModuleID::LEFT_BACK:
        m_nt = string("LeftBackSwerveModule");
        break;

    case ModuleID::RIGHT_FRONT:
        m_nt = string("RightFrontSwerveModule");
        break;

    case ModuleID::RIGHT_BACK:
        m_nt = string("RightBackSwerveModule");
        break;

    default:
        m_nt = string("UnknownSwerveModule");
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_nt, string("SwerveModuleDrive"), string("unknown module"));
        break;
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "CurrentPoseX", to_string(m_currentPose.X().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "CurrentPoseY", to_string(m_currentPose.Y().to<double>()));
}

/// @brief initialize the swerve module with information that the swerve chassis knows about
/// @param [in] units::velocity::meters_per_second_t - maxVelocity: maximum linear velocity of the chassis
/// @param [in] units::angular_velocity::radians_per_second_t - maxAngularVelocity: maximum angular velocity of the chassis
/// @param [in] units::acceleration::meters_per_second_squared_t - maxAcceleration: maximum linear acceleration of the chassis
/// @param [in] units::angular_acceleration::radians_per_second_squared_t - maxAngularAcceleration: maximum angular acceleration of the chassis

void SwerveModule::Init(units::velocity::meters_per_second_t maxVelocity,
                        units::angular_velocity::radians_per_second_t maxAngularVelocity,
                        units::acceleration::meters_per_second_squared_t maxAcceleration,
                        units::angular_acceleration::radians_per_second_squared_t maxAngularAcceleration,
                        Translation2d offsetFromCenterOfRobot)
{
    m_maxVelocity = maxVelocity;
    m_driveVelocityControlData = ControlData(ControlModes::CONTROL_TYPE::VELOCITY_RPS,
                                             ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
                                             string("DriveSpeed"),
                                             0.01, // 0.01
                                             0.0,
                                             0.0,
                                             0.5, // 0.5
                                             ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE,
                                             0.0,
                                             maxAcceleration.to<double>(),
                                             maxVelocity.to<double>(),
                                             maxVelocity.to<double>(),
                                             0.0,
                                             false);
    m_driveMotor->SetControlConstants(0, m_runClosedLoopDrive ? m_driveVelocityControlData : m_drivePercentControlData);
    // auto trans = Transform2d(offsetFromCenterOfRobot, Rotation2d() );
    // m_currentPose = m_currentPose + trans;
}

/// @brief Set all motor encoders to zero
/// @brief void
void SwerveModule::SetEncodersToZero()
{
    // Add method to set position to 0 and add to drive motor
}

/// @brief Get the encoder values
/// @returns double - the integrated sensor position
double SwerveModule::GetEncoderValues()
{
    return m_driveMotor->GetCounts();
}

/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    // Desired State
    SetTurnAngle(units::degree_t(0));
}

/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * numbers::pi);
    auto mps = units::velocity::meters_per_second_t(mpr.to<double>() * m_driveMotor->GetRPS());

    // Get the Module Current Rotation Angle
    Rotation2d angle{m_turnSensor->GetAbsolutePosition()};

    // Create the state and return it
    SwerveModuleState state{mps, angle};
    return state;
}

/// @brief Get the current position of the swerve module (distance and rotation)
/// @return frc::SwerveModulePosition - current position
frc::SwerveModulePosition SwerveModule::GetPosition() const
{
    return {m_driveMotor->GetRotations() * m_wheelDiameter * numbers::pi, // distance travled by drive motor
            Rotation2d(m_turnSensor->GetAbsolutePosition())};             // angle of the swerve module from sensor
}

/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState(
    const SwerveModuleState &targetState)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    Rotation2d currAngle = Rotation2d(m_turnSensor->GetAbsolutePosition());
    auto optimizedState = Optimize(targetState, currAngle);
    // auto optimizedState = SwerveModuleState::Optimize(targetState, currAngle);
    // auto optimizedState = targetState;

    // Set Turn Target
    SetTurnAngle(optimizedState.angle.Degrees());

    // Set Drive Target
    SetDriveSpeed(optimizedState.speed);
}

/// @brief Given a desired swerve module state and the current angle of the swerve module, determine
///        if the changing the desired swerve module angle by 180 degrees is a smaller turn or not.
///        If it is, return a state that has that angle and the reversed speed.  Otherwise, return the
///        original desired state.
/// Note:  the following was taken from the WPI code and tweaked because we were seeing some weird
///        reversals that we believe was due to not using a tolerance
/// @param [in] const SwerveModuleState& desired state of the swerve module
/// @param [in] const Rotation2d& current angle of the swerve module
/// @returns SwerveModuleState optimized swerve module state
SwerveModuleState SwerveModule::Optimize(
    const SwerveModuleState &desiredState,
    const Rotation2d &currentAngle)
{
    SwerveModuleState optimizedState;
    optimizedState.angle = desiredState.angle;
    optimizedState.speed = desiredState.speed;

    auto delta = AngleUtils::GetDeltaAngle(currentAngle.Degrees(), optimizedState.angle.Degrees());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "Optimize current", currentAngle.Degrees().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "Optimize target", optimizedState.angle.Degrees().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "Optimize delta", delta.to<double>());

    // deal with roll over issues (e.g. want to go from -180 degrees to 180 degrees or vice versa)
    // keep the current angle
    if ((units::math::abs(delta) > 359_deg))
    {
        optimizedState.angle = currentAngle.Degrees();
    }
    // if delta is > 90 degrees or < -90 degrees, we can turn the wheel the otherway and
    // reverse the wheel direction (negate speed)
    // if the delta is > 90 degrees, rotate the module the opposite direction and negate the speed
    else if ((units::math::abs(delta)) > 90_deg)
    {
        optimizedState.speed *= -1.0;
        optimizedState.angle = optimizedState.angle + Rotation2d{180_deg};
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "Optimize reversing", delta.to<double>());
    }

    // if the delta is > 90 degrees, rotate the opposite way and reverse the wheel
    if ((units::math::abs(delta) - 90_deg) > 0.1_deg)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "optimized", (desiredState.angle + Rotation2d{180_deg}).Degrees().to<double>());
        return {-desiredState.speed, desiredState.angle + Rotation2d{180_deg}};
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "optimized", desiredState.angle.Degrees().to<double>());
        return {desiredState.speed, desiredState.angle};
    }
}

/// @brief Run the swerve module at the same speed and angle
/// @returns void
void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_activeState.speed);

    // TODO: add method to stop motor and apply to turn motor
}

/// @brief run the drive motor at a specified speed
/// @param [in] speed to drive the drive wheel as
/// @returns void
void SwerveModule::SetDriveSpeed(units::velocity::meters_per_second_t speed)
{
    m_activeState.speed = (abs(speed.to<double>() / m_maxVelocity.to<double>()) < 0.05) ? 0_mps : speed;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("State Speed - mps"), m_activeState.speed.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("Wheel Diameter - meters"), units::length::meter_t(m_wheelDiameter).to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("drive motor id"), m_driveMotor->GetID());

    if (m_runClosedLoopDrive)
    {
        // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
        auto driveTarget = m_activeState.speed.to<double>() / (units::length::meter_t(m_wheelDiameter).to<double>() * numbers::pi);
        driveTarget /= m_driveMotor->GetGearRatio();
        m_driveMotor->Set(driveTarget);
    }
    else
    {
        double percent = m_activeState.speed / m_maxVelocity;
        m_driveMotor->Set(percent);
    }
}

/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetTurnAngle(units::angle::degree_t targetAngle)
{
    m_activeState.angle = targetAngle;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("turn motor id"), m_turnMotor->GetID());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("target angle"), targetAngle.to<double>());

    auto deltaAngle = AngleUtils::GetDeltaAngle(m_turnSensor->GetAbsolutePosition(), targetAngle);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("current angle"), m_turnSensor->GetAbsolutePosition().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("delta angle"), deltaAngle.to<double>());

    if (abs(deltaAngle.to<double>()) > 1.0)
    {
        // TODO: re-work logic
        /**
        auto motor = m_turnMotor->GetSpeedController();
        auto fx = dynamic_cast<WPI_TalonFX *>(motor);
        auto sensors = fx->GetSensorCollection();
        **/
        // auto deltaTicks = m_countsOnTurnEncoderPerDegreesOnAngleSensor * deltaAngle.to<double>();

        // double currentTicks = sensors.GetIntegratedSensorPosition();
        // double desiredTicks = currentTicks + deltaTicks;

        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("currentTicks"), currentTicks);
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("deltaTicks"), deltaTicks);
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, string("desiredTicks"), desiredTicks);

        // m_turnMotor->Set(desiredTicks);
    }
}

/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    // TODO: add method to stop motor and do it for both turn and drive motors
}

frc::Pose2d SwerveModule::GetCurrentPose(PoseEstimatorEnum opt)
{
    // get the information from the last pose
    auto startX = m_currentPose.X();
    auto startY = m_currentPose.Y();
    auto startAngle = m_currentPose.Rotation().Radians();
    auto startRotations = m_currentRotations;

    // read sensor info (cancoder, encoders) for current speed and angle of the module
    // calculate the average from the last
    auto currentAngle = m_turnSensor->GetAbsolutePosition();
    auto currentRotations = m_driveMotor->GetRotations();

    units::length::meter_t currentX{units::length::meter_t(0)};
    units::length::meter_t currentY{units::length::meter_t(0)};

    if (opt == PoseEstimatorEnum::EULER_USING_MODULES)
    {
        // Euler Method
        //
        // xk+1 = xk + vk cos θk T = xk + delta * cos θk
        // yk+1 = yk + vk sin θk T = yk + delta * sin θk
        // Thetak+1 = Thetagyro,k+1

        auto delta = currentRotations - startRotations;
        auto circum = numbers::pi * m_wheelDiameter;

        currentX = startX + cos(startAngle.to<double>()) * circum;
        currentY = startY + sin(startAngle.to<double>()) * circum;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "start rotations", startRotations);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "current rotations", currentRotations);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "delta", delta);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "circumference", circum.to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "WheelDiameter", m_wheelDiameter.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "CurrentX", currentX.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "CurrentY", currentY.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "startX", startX.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "startY", startY.to<double>());
    }
    else if (opt == PoseEstimatorEnum::POSE_EST_USING_MODULES)
    {
        // Pose estimation method from Controls Engineering in FIRST Robotics Competition by
        // Tyler Veness
        //
        // variables:
        //   dX - delta X - Change in pose's X (forward/backward directions)
        //   dY - delta Y - Change in pose's Y (strafe directions)
        //   dT - delta theta - Change in pose's theta (rotation about its center of rotation)
        //    T - theta - Starting Angle in global coordinate frame
        //    t - time since last pose update
        //    vx - velocity along X-axis
        //    vy - velocity along Y-axis
        //    w - omega - angular velocity
        //
        //   Matrices with a G are in global space and R are in Robot space
        //
        //  G -    -       -                     - R-                                  - R-    -
        //    | dX |       | cos(T)  -sin(T)   0 |  | sin(wt) / w     cos(wt)-1 / w  0 |  | vx |
        //    | dY |   =   | sin(T)   cos(T)   0 |  | 1-cos(wt) / w   sin(wt) / w    0 |  | vy |
        //    | dt |       |   0       0       1 |  |      0               0         t |  | w  |
        //    -    -       -                     -  -                                  -  -    -
        //
        // or
        //
        //  G -    -       -                     - R-                                    - R-    -
        //    | dX |       | cos(T)  -sin(T)   0 |  | sin(dT) / dT     cos(dT)-1 / dT  0 |  | dX |
        //    | dY |   =   | sin(T)   cos(T)   0 |  | 1-cos(dT) / dT   sin(dT) / dT    0 |  | dY |
        //    | dt |       |   0       0       1 |  |      0               0           1 |  | dY |
        //    -    -       -                     -  -                                    -  -    -
        // TODO:  if Euler isn't accurate enough implement this
        // If so, can we use the Euler matrix library to do this or do we need to manually do it ourselves?
    }

    // update the pose (do we need to look if our encoder distance matches and adjust here???)
    auto newpose = Pose2d(currentX, currentY, Rotation2d(currentAngle));
    auto trans = newpose - m_currentPose;
    m_currentPose = m_currentPose + trans;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "NewPoseX", newpose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "NewPoseY", newpose.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "TransX", trans.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_nt, "TransY", trans.Y().to<double>());

    m_currentRotations = currentRotations;
    return m_currentPose;
}

void SwerveModule::UpdateCurrPose(
    units::length::meter_t x,
    units::length::meter_t y)
{
    m_currentPose = m_currentPose + Transform2d{Translation2d{x, y}, Rotation2d{m_turnSensor->GetAbsolutePosition()}};
}