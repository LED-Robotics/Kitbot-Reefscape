// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;
using namespace rev::spark;

class SwerveModule {
    public:
        SwerveModule(hardware::TalonFX *drivingMotor, hardware::TalonFX *turningMotor);
        
        SwerveModule(hardware::TalonFX *drivingMotor, SparkMax *turningMotor, 
        DutyCycleEncoder *thetaEncoder, double thetaEncoderOffset = 0, bool flipEncoder = false);
        /**
         * Gets the distance of the drive encoder.
         *
         * @return the drive encoder distance
         */
        units::meter_t GetDriveEncoderDistance() const;

        /**
         * Gets the distance of the turn encoder.
         *
         * @return the turn encoder distance
         */
        units::degree_t GetTurnEncoderAngle() const;

        /**
         * Gets the rate of the encoder.
         *
         * @return the encoder distance
         */
        units::meters_per_second_t GetDriveEncoderRate() const;
        /**
         * Gets the current state of the swerve module.
         *
         * @return a SwerveModuleState representing the module
         */
        frc::SwerveModuleState GetState() const;
        /**
         * Gets the current position of the swerve module.
         *
         * @return a SwerveModulePosition representing the module
         */
        frc::SwerveModulePosition GetPosition() const;
        /**
         * Corrects an angle with a non-continuous range and sets it to 0-360.
         *
         * @return the angle in its corrected range
         */
        static double PlaceInAppropriate0To360Scope(double scopeReference, double newAngle);
        /**
         * Optimize swerve module target to minimize unnecessary movement.
         *
         * @return the optimized SwerveModuleState
         */
        static frc::SwerveModuleState Optimize(const frc::SwerveModuleState& desiredState, frc::Rotation2d currentAngle);
        /**
         * Sets the state of the swerve module.
         */
        void SetDesiredState(const frc::SwerveModuleState& state);
        /**
         * Run one cycle of theta PID Controller. This should be run repeatedly while operational.
         */
        void RunPID();
        /**
         * Debug function to set swerve drive motor using power.
         */
        void SetDrivePower(double power);
        /**
         * Debug function to set swerve turn motor using power.
         */
        void SetTurnPower(double power);
        /**
         * Resets the module motors' encoders.
         */
        void ResetEncoders();

    private:

        double GetFalconTurnPosition() const; 
        double GetNeoTurnPosition() const; 

        void SetFalconTurnPower(double power); 
        void SetNeoTurnPower(double power); 

        // motor references
        bool usingFalcon = true;
        hardware::TalonFX *driveMotor;
        hardware::TalonFX *falconTurn;
        controls::VelocityVoltage velocity{0_tps};
        controls::PositionVoltage rotation{0_tr};

        SparkMax *neoTurn;
        DutyCycleEncoder *neoEncoder;
        // Only for DutyCycle
        double turnEncoderOffset = 0.0;
        // frc2::PIDController neoController{0.005, 0.0, 0.0}; off floor
        frc::PIDController neoController{0.01, 0.0, 0.0};
        bool encoderFlipped = false;
        
};
