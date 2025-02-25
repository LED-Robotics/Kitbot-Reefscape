// Copyright (c) FIRST and other WPILib contributors.
// Open Soruce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in root directory of this project.

#include "subsystems/DriveSubsystem/SwerveModule.h"

#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(hardware::TalonFX *drivingMotor,
                            hardware::TalonFX *turningMotor) {
    driveMotor = drivingMotor;
    falconTurn = turningMotor;
}

SwerveModule::SwerveModule(hardware::TalonFX *drivingMotor, SparkMax *turningMotor, 
        DutyCycleEncoder *thetaEncoder, double thetaEncoderOffset) {
    usingFalcon = false;
    driveMotor = drivingMotor;
    neoTurn = turningMotor;
    neoEncoder = thetaEncoder;
    turnEncoderOffset = thetaEncoderOffset;
}

double SwerveModule::GetFalconTurnPosition() const {
    return falconTurn->GetPosition().GetValueAsDouble() * 360.0;
}

double SwerveModule::GetNeoTurnPosition() const {
    double angle = turnEncoderOffset - neoEncoder->Get();
    return angle * DriveConstants::kTurnEncoderDegreesPerPulse;
}

void SwerveModule::SetFalconTurnPower(double power) {
    falconTurn->Set(power);
}

void SwerveModule::SetNeoTurnPower(double power) {
    neoTurn->Set(power);
}

units::meter_t SwerveModule::GetDriveEncoderDistance() const {
    return units::meter_t{driveMotor->GetPosition().GetValueAsDouble() * DriveConstants::kDriveDistancePerRev};
}

units::degree_t SwerveModule::GetTurnEncoderAngle() const {
    return units::degree_t{usingFalcon ? GetFalconTurnPosition() : GetNeoTurnPosition()};
}

units::meters_per_second_t SwerveModule::GetDriveEncoderRate() const {
    return units::meters_per_second_t{driveMotor->GetVelocity().GetValueAsDouble() * DriveConstants::kDriveDistancePerRev};
}

frc::SwerveModuleState SwerveModule::GetState() const {
    return {GetDriveEncoderRate(),
            GetTurnEncoderAngle()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {GetDriveEncoderDistance(),
            GetTurnEncoderAngle()};
}

// adapted from team 364s BaseFalconSwerve example
frc::SwerveModuleState SwerveModule::Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle) {
    double targetAngle = PlaceInAppropriate0To360Scope((double)currentAngle.Degrees(), (double)desiredState.angle.Degrees());
    double targetSpeed = (double)desiredState.speed;
    double delta = targetAngle - (double)currentAngle.Degrees();
    if (fabs(delta) > 90.0){
        targetSpeed *= -1.0;
        targetAngle = delta > 90.0 ? (targetAngle - 180.0) : (targetAngle + 180.0);
    }
    return frc::SwerveModuleState{units::velocity::meters_per_second_t{targetSpeed}, {units::degree_t{targetAngle}}};
}

// adapted from team 364s BaseFalconSwerve example
double SwerveModule::PlaceInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = fmod(scopeReference, 360.0);
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360.0 - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360.0 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360.0;
    }
    while (newAngle > upperBound) {
        newAngle -= 360.0;
    }
    if (newAngle - scopeReference > 180.0) {
        newAngle -= 360.0;
    } else if (newAngle - scopeReference < -180.0) {
        newAngle += 360.0;
    }
    return newAngle;
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

    // Optimize the reference state to avoid spinning further than 90 degrees*
    const auto state = Optimize(referenceState, {GetTurnEncoderAngle()});
    
    if(usingFalcon) {
        falconTurn->SetControl(rotation.WithPosition(units::angle::turn_t{(double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse}));
    } else {
        // frc::SmartDashboard::PutNumber("BL Target", (double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);
        // frc::SmartDashboard::PutNumber("Target", (double)referenceState.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);
        // std::cout << "Neo Target: " << (double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse << '\n';
        // neoController->SetReference((double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse, SparkMax::ControlType::kPosition);
        // neoController->SetReference((double)referenceState.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse, SparkMax::ControlType::kPosition);
        // neoController.SetSetpoint((double)state.angle.Degrees() / DriveConstants::kTurnEncoderDegreesPerPulse);
        neoController.SetSetpoint((double)state.angle.Degrees());
        // frc::SmartDashboard::PutNumber("FL Target", neoController.GetSetpoint());
    }

    driveMotor->SetControl(velocity.WithVelocity(
        units::angular_velocity::turns_per_second_t{((double)state.speed) / DriveConstants::kDriveDistancePerRev}));
}

void SwerveModule::RunPID() {
    double angle = (double)GetTurnEncoderAngle();
    double power = neoController.Calculate(angle);
    // frc::SmartDashboard::PutNumber("FL Angle", angle);
    // frc::SmartDashboard::PutNumber("FL Power", power);

    neoTurn->Set(power);
}

// debug
void SwerveModule::SetDrivePower(double power) {
    driveMotor->Set(power);
}

void SwerveModule::SetTurnPower(double power) {
    if(usingFalcon) SetFalconTurnPower(power);
    else SetNeoTurnPower(power);
    // turnMotor->Set(power);
}

void SwerveModule::ResetEncoders() {
    driveMotor->SetPosition(units::angle::turn_t{0.0});
    if(usingFalcon) falconTurn->SetPosition(units::angle::turn_t{0.0});
    /*else neoEncoder->Reset();*/
}
