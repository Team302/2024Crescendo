
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
#include "frc/controller/PIDController.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "chassis/PoseEstimatorEnum.h"
#include "chassis/swerve/SwerveChassis.h"
#include "chassis/swerve/SwerveModule.h"
#include "chassis/swerve/SwerveModuleConstants.h"
#include "hw/DragonCanCoder.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/ControlModes.h"
#include "utils/AngleUtils.h"
#include "utils/logging/Logger.h"

// Third Party Includes
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"

using namespace std;
using namespace frc;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;

/// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
/// @param [in] SwerveModuleConstants.ModuleID                                                type:           Which Swerve Module is it
/// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move
/// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module
/// @param [in] DragonCanCoder*                                 		canCoder:       Sensor for detecting the angle of the wheel
/// @param [in] units::length::inch_t                                   wheelDiameter   Diameter of the wheel
SwerveModule::SwerveModule(SwerveModuleConstants::ModuleID id,
                           SwerveModuleConstants::ModuleType type,
                           IDragonMotorController *driveMotor,
                           IDragonMotorController *turnMotor,
                           DragonCanCoder *canCoder) : m_moduleID(id),
                                                       m_driveMotor(driveMotor),
                                                       m_turnMotor(turnMotor),
                                                       m_turnSensor(canCoder),
                                                       m_activeState(),
                                                       m_currentPose(),
                                                       m_currentSpeed(0.0_rpm),
                                                       m_currentRotations(0.0)
{
    Rotation2d ang{units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;

    auto attrs = SwerveModuleConstants::GetSwerveModuleAttrs(type);

    auto driveTalon = dynamic_cast<DragonTalonFX *>(m_driveMotor);
    if (driveTalon != nullptr)
    {
        driveTalon->SetControlConstants(0, attrs.driveControl);
    }

    m_turnSensor->SetRange(AbsoluteSensorRangeValue::Signed_PlusMinusHalf);

    auto turnTalon = dynamic_cast<DragonTalonFX *>(m_turnMotor);
    if (turnTalon != nullptr)
    {
        turnTalon->FuseCancoder(*canCoder, attrs.sensorToMechanismRatio, attrs.rotorToSensorRatio);
        turnTalon->SetControlConstants(0, attrs.angleControl);
    }
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
void SwerveModule::SetDesiredState(const SwerveModuleState &targetState)
{
    UpdateTuningParms();

    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90
    Rotation2d currAngle = Rotation2d(m_turnSensor->GetAbsolutePosition());
    auto optimizedState = Optimize(targetState, currAngle);

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
SwerveModuleState SwerveModule::Optimize(const SwerveModuleState &desiredState,
                                         const Rotation2d &currentAngle)
{
    SwerveModuleState optimizedState;
    optimizedState.angle = desiredState.angle;
    optimizedState.speed = desiredState.speed;

    auto delta = AngleUtils::GetDeltaAngle(currentAngle.Degrees(), optimizedState.angle.Degrees());

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
    }

    // if the delta is > 90 degrees, rotate the opposite way and reverse the wheel
    if ((units::math::abs(delta) - 90_deg) > 0.1_deg)
    {
        return {-desiredState.speed, desiredState.angle + Rotation2d{180_deg}};
    }
    else
    {
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
    m_turnMotor->Set(targetAngle.to<double>());
}

/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    // TODO: add method to stop motor and do it for both turn and drive motors
}

std::shared_ptr<nt::NetworkTable> SwerveModule::GetNetworkTable()
{
    auto ntinstance = nt::NetworkTableInstance::GetDefault();
    return ntinstance.GetTable("SwerveModuleAttrs");
}

void SwerveModule::InitTuningParms(const SwerveModuleAttributes &attrs)
{
    m_turnControl = attrs.angleControl.GetMode();
    m_turnKp = attrs.angleControl.GetP();
    m_turnKi = attrs.angleControl.GetI();
    m_turnKd = attrs.angleControl.GetD();
    m_turnKf = attrs.angleControl.GetF();
    m_turnCruiseVel = attrs.angleControl.GetCruiseVelocity();
    m_turnMaxAcc = attrs.angleControl.GetMaxAcceleration();

    m_driveControl = attrs.driveControl.GetMode();
    m_driveKp = attrs.driveControl.GetP();
    m_driveKi = attrs.driveControl.GetI();
    m_driveKd = attrs.driveControl.GetD();
    m_driveKf = attrs.driveControl.GetF();
}
void SwerveModule::DisplayTuningParms()
{
    auto ntentry = GetNetworkTable();
    m_tkp = ntentry->GetDoubleTopic("turn P").Subscribe(m_turnKp);
    m_tki = ntentry->GetDoubleTopic("turn I").Subscribe(m_turnKi);
    m_tkd = ntentry->GetDoubleTopic("turn D").Subscribe(m_turnKd);
    m_tkf = ntentry->GetDoubleTopic("turn F").Subscribe(m_turnKf);
    m_tmvel = ntentry->GetDoubleTopic("turn Cruise Vel").Subscribe(m_turnKf);
    m_tmacc = ntentry->GetDoubleTopic("turn Max Accel").Subscribe(m_turnKf);

    m_dkp = ntentry->GetDoubleTopic("drive P").Subscribe(m_driveKp);
    m_dki = ntentry->GetDoubleTopic("drive I").Subscribe(m_driveKi);
    m_dkd = ntentry->GetDoubleTopic("drive D").Subscribe(m_driveKd);
    m_dkf = ntentry->GetDoubleTopic("drive F").Subscribe(m_driveKf);
}

void SwerveModule::UpdateTuningParms()
{
    bool needToUpdate = false;
    auto ntentry = GetNetworkTable();

    auto value = m_tkp.Get();
    if (value != m_turnKp)
    {
        needToUpdate = true;
        m_turnKp = value;
    }

    value = m_tki.Get();
    if (value != m_turnKi)
    {
        needToUpdate = true;
        m_turnKi = value;
    }

    value = m_tkd.Get();
    if (value != m_turnKd)
    {
        needToUpdate = true;
        m_turnKd = value;
    }

    value = m_tkf.Get();
    if (value != m_turnKf)
    {
        needToUpdate = true;
        m_turnKf = value;
    }

    value = m_tmacc.Get();
    if (value != m_turnMaxAcc)
    {
        needToUpdate = true;
        m_turnMaxAcc = value;
    }

    value = m_tmvel.Get();
    if (value != m_turnCruiseVel)
    {
        needToUpdate = true;
        m_turnCruiseVel = value;
    }

    value = m_dkp.Get();
    if (value != m_driveKp)
    {
        needToUpdate = true;
        m_driveKp = value;
    }

    value = m_dki.Get();
    if (value != m_driveKi)
    {
        needToUpdate = true;
        m_driveKi = value;
    }

    value = m_dkd.Get();
    if (value != m_driveKd)
    {
        needToUpdate = true;
        m_driveKd = value;
    }

    value = m_dkf.Get();
    if (value != m_driveKf)
    {
        needToUpdate = true;
        m_driveKf = value;
    }

    if (needToUpdate)
    {
        ControlData turncd;
        turncd.SetMode(m_turnControl);
        turncd.SetP(m_turnKp);
        turncd.SetI(m_turnKi);
        turncd.SetD(m_turnKd);
        turncd.SetF(m_turnKf);
        turncd.SetMaxAcceleration(m_turnMaxAcc);
        turncd.SetCruiseVelocity(m_turnCruiseVel);
        auto turnTalon = dynamic_cast<DragonTalonFX *>(m_turnMotor);
        if (turnTalon != nullptr)
        {
            turnTalon->SetControlConstants(0, turncd);
        }

        ControlData drivecd;
        drivecd.SetMode(m_driveControl);
        drivecd.SetP(m_driveKp);
        drivecd.SetI(m_driveKi);
        drivecd.SetD(m_driveKd);
        drivecd.SetF(m_driveKf);

        auto driveTalon = dynamic_cast<DragonTalonFX *>(m_driveMotor);
        if (driveTalon != nullptr)
        {
            driveTalon->SetControlConstants(0, drivecd);
        }
    }
}
