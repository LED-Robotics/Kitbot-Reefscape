// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CascadeConstants;
using namespace frc;

CascadeSubsystem::CascadeSubsystem()
  : motor{kMotorPort, rev::spark::SparkLowLevel::MotorType::kBrushless}
  // encoder{kEncoderPort} 
  {
    SmartDashboard::PutNumber("Cascade Position", position.value());
    /*SmartDashboard::PutNumber("Cascade Power", 0.0);*/
    ConfigMotors();
    SetTargetPosition(0.0_m);

}

void CascadeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetTargetPosition(units::length::meter_t{SmartDashboard::GetNumber("Cascade Position", position.value())});
  /*SetPower(SmartDashboard::GetNumber("Cascade Power", power));*/
  /*SmartDashboard::PutNumber("Cascade Voltage", left.GetMotorVoltage().GetValueAsDouble());*/
  SmartDashboard::PutNumber("Actual Cascade", GetPosition().value());
  if(state == kOff) {
    motor.Set(0.0);
  } else if(state == CascadeStates::kPowerMode) {
    motor.Set(power);
  } else if(state == CascadeStates::kPositionMode) {

  SmartDashboard::PutNumber("cascadeTr", motor.GetEncoder().GetPosition());
    
    units::angle::turn_t posTarget{(position - kStartPosition).value() * kTurnsPerMeter};
    SmartDashboard::PutNumber("PositionTr", posTarget.value());
    motor.GetClosedLoopController().SetReference(posTarget.value(), SparkBase::ControlType::kPosition);
    // left.SetControl(positionController
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
    // right.SetControl(positionController
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));

    // Test Motion Magic
    // left.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
    // right.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
  }
}

void CascadeSubsystem::Off() {
  state = CascadeStates::kOff;
}

void CascadeSubsystem::On() {
  state = CascadeStates::kPowerMode;
}

void CascadeSubsystem::SetPower(double newPower) {
  power = newPower;
}

void CascadeSubsystem::SetState(int newState) {
  state = newState;
}

int CascadeSubsystem::GetState() {
  return state;
}

units::length::meter_t CascadeSubsystem::GetPosition() {
  return units::length::meter_t{motor.GetEncoder().GetPosition() / kTurnsPerMeter} + kStartPosition;
}

void CascadeSubsystem::SetTargetPosition(units::length::meter_t newPosition) {
  position = newPosition;
  if(position < kCascadeMeterMin) position = kCascadeMeterMin;
  if(position > kCascadeMeterMax) position = kCascadeMeterMax;
}

bool CascadeSubsystem::IsAtTarget() {
  auto target = position;
  auto pos = GetPosition();
  return pos > target - (kPositionDeadzone / 2) && pos < target + (kPositionDeadzone / 2);
}

void CascadeSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  // motor.GetConfigurator().Apply(updated, 50_ms);
  // right.GetConfigurator().Apply(updated, 50_ms);
}

void CascadeSubsystem::ConfigMotors() {
  SparkMaxConfig config{};
  config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
  config.SmartCurrentLimit(40.0);
  config.Inverted(false);
  ClosedLoopConfig closedLoop;
  closedLoop.SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
  closedLoop.P(0.05);
  closedLoop.I(0.0);
  closedLoop.D(0.0);
  closedLoop.OutputRange(-1.0, 1.0);
  config.Apply(closedLoop);

  motor.Configure(config, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}

frc2::CommandPtr CascadeSubsystem::GetMoveCommand(units::length::meter_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetPosition(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
}
