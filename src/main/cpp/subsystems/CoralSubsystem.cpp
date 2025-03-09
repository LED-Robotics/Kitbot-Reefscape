// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem/CoralSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>


using namespace CoralConstants;
using namespace frc;

CoralSubsystem::CoralSubsystem()
  : wristMotor{kWristPort, SparkLowLevel::MotorType::kBrushless},
    intakeMotor{kIntakePort, SparkLowLevel::MotorType::kBrushless} {
      /*wristMotor.SetPosition(0.0_tr);*/
      SmartDashboard::PutNumber("Coral Angle", kWristStartAngle.value());

      // ConfigIntake();
      ConfigWrist();

      /*SetTargetAngle(kWristStartAngle);*/

}

void CoralSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Wrist Control
  // SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Coral Angle", GetAngle().value())});
  SmartDashboard::PutNumber("Coral Actual", GetAngle().value());
  if(wristState == WristStates::kWristOff) {
    wristMotor.Set(0.0);
  } else if(wristState == WristStates::kWristPowerMode) {
    wristMotor.Set(wristPower);
  } else if(wristState == WristStates::kWristAngleMode) {
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.


    SmartDashboard::PutNumber("coralWristTr", wristMotor.GetEncoder().GetPosition());  // print to Shuffleboard
    SmartDashboard::PutNumber("coralAngle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(wristAngle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("Coral Target", wristAngle.value());
    units::angle::turn_t posTarget{(wristAngle - kWristStartAngle).value() * kTurnsPerDegree};
    SmartDashboard::PutNumber("coralTrTarget", posTarget.value());
    wristMotor.GetClosedLoopController().SetReference(posTarget.value(), SparkBase::ControlType::kPosition);

    // Intake Control
    if(intakeState == IntakeStates::kIntakeOff) {
      intakeMotor.Set(0.0);
    } else {
      intakeMotor.Set(intakePower);
    }
  }
}

void CoralSubsystem::IntakeOn() {
  intakeState = IntakeStates::kIntakePowerMode;
}

void CoralSubsystem::IntakeOff() {
  intakeState = IntakeStates::kIntakeOff;
}

void CoralSubsystem::SetIntakePower(double newPower) {
  intakePower = newPower;
}

double CoralSubsystem::GetIntakePower() {
  return intakePower;
}

void CoralSubsystem::SetIntakeState(int newState) {
  intakeState = newState;
}

int CoralSubsystem::GetIntakeState() {
  return intakeState;
}

// void AlgaeSubsystem::SetIntakeBrakeMode(bool state) {
//   signals::NeutralModeValue mode;
//   if(state) mode = signals::NeutralModeValue::Brake;
//   else mode = signals::NeutralModeValue::Coast;
//   configs::MotorOutputConfigs updated;
//   updated.WithNeutralMode(mode);
//   intakeMotor.GetConfigurator().Apply(updated, 50_ms);
// }

// void AlgaeSubsystem::ConfigIntake() {
//   configs::TalonFXConfiguration algaeIntakeConfig{};

//   algaeIntakeConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
//   algaeIntakeConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
//   algaeIntakeConfig.MotorOutput.Inverted = true;

//   intakeMotor.GetConfigurator().Apply(algaeIntakeConfig);
// }

bool CoralSubsystem::IsCoralIndexed() {
  // Add when limit switch is added
  return false;
}

void CoralSubsystem::WristOn() {
  wristState = WristStates::kWristAngleMode;
}

void CoralSubsystem::WristOff() {
  wristState = WristStates::kWristOff;
}

void CoralSubsystem::SetWristPower(double newPower) {
  wristPower = newPower;
}

double CoralSubsystem::GetWristPower() {
  return wristPower;
}

void CoralSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  wristAngle = newAngle;
  if(wristAngle < kWristDegreeMin) wristAngle = kWristDegreeMin;
  if(wristAngle > kWristDegreeMax) wristAngle = kWristDegreeMax;
}

units::angle::degree_t CoralSubsystem::GetAngle() {
  return units::angle::degree_t{(GetWristPosition() / kTurnsPerDegree)} + kWristStartAngle;
}

double CoralSubsystem::GetWristPosition() {
  return wristMotor.GetEncoder().GetPosition();
}

bool CoralSubsystem::IsAtTarget() {
  auto target = wristAngle;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kWristAngleDeadzone / 2) && angle < target + (kWristAngleDeadzone / 2);
  return atTarget;
}

void CoralSubsystem::SetWristState(int newState) {
  wristState = newState;
}

int CoralSubsystem::GetWristState() {
  return wristState;
}

frc2::CommandPtr CoralSubsystem::GetMoveCommand(units::angle::degree_t target) {
  /*return frc2::cmd::Sequence(*/
  /*    frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this}),*/
  /*    frc2::cmd::WaitUntil([this, target](){*/
  /*      return IsAtTarget();*/
  /*    }));*/
  return frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this});
}
void CoralSubsystem::SetWristBrakeMode(bool state) {

}

void CoralSubsystem::ConfigWrist() {
  wristConfig
    .SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
    .SmartCurrentLimit(40.0)
    .Inverted(true)
  .closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .P(0.1)
    .I(0.0)
    .D(0.0)
    .OutputRange(-1.0, 1.0);

  wristMotor.Configure(wristConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
}
