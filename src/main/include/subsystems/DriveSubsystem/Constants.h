#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <frc/geometry/Pose2d.h>


namespace DriveConstants {
    //Wheel motors
    constexpr int kBackLeftPort = 0;
    constexpr int kFrontLeftPort = 1;
    constexpr int kBackRightPort = 2;
    constexpr int kFrontRightPort = 3;
    //Degree of wheel motors
    constexpr int kBackLeftThetaPort = 4;
    constexpr int kFrontLeftThetaPort = 5;
    constexpr int kBackRightThetaPort = 6;
    constexpr int kFrontRightThetaPort = 7;

    constexpr int kBackLeftEncoderPort = 3;
    constexpr int kFrontLeftEncoderPort = 2;
    constexpr int kBackRightEncoderPort = 8;
    constexpr int kFrontRightEncoderPort = 0;
    
    constexpr double kBLeftMagPos = 0.505863762646594;
    constexpr double kFLeftMagPos = 0.253170756329269;
    constexpr double kBRightMagPos = 0.847834921195873 - 0.5;
    constexpr double kFRightMagPos = 0.882721822068046;
    
    // info for odometry
    constexpr int kEncoderResolution = 4096;
    constexpr auto kDriveBaseRadius = 0.319786_m;
    constexpr double kWheelRadius = 0.0381;
    constexpr double kDriveRatio = 1 / 4.13;
    constexpr double kTurnRatio = 1 / 10.29;
    constexpr double kDriveDistancePerRev = (2 * std::numbers::pi * kWheelRadius) * kDriveRatio;
    // constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / (double)kEncoderResolution) * kDriveRatio;
    // constexpr double kTurnEncoderDegreesPerPulse = (360.0 / (double)kEncoderResolution) * kTurnRatio;
    constexpr double kTurnEncoderDegreesPerPulse = 360.0;

    // states
    constexpr int kArbitrary = 0;

    // TalonFX config
    constexpr double kDriveP = 0.08;
    constexpr double kDriveV = 0.1345313787460327;
    constexpr units::time::second_t kDriveRamp = 0.0_s;
    constexpr units::current::ampere_t kDriveCurrentLimit = 30.0_A;
    constexpr double kTurnP = 80.0;
    constexpr double kTurnPRatio = 10.279000282287598;

    constexpr auto turnKp = 0.275;

    // drivetrain controller constants
    constexpr double kDriveDeadzone = 0.2;
    constexpr double kTurnDeadzone = 0.1;
    constexpr double kDriveCurveExtent = 0.6;
    constexpr auto kDriveAccelerationLimit = 9.0_mps_sq;
    constexpr auto kDriveDecelerationLimit = 7.0_mps_sq;
    constexpr auto kDriveTranslationLimit = 3.3_mps;
    constexpr double kTxAdjust = 0.5;
    constexpr double kAlignP = 0.5;
    constexpr double kPVelTurnOffset = -2.0;
    constexpr double kPVelDistOffset = 0.0;
    constexpr double kDistMultiplier = 1.5;
    constexpr int kDistSamples = 5;
    constexpr double kPNote = 0.3;
    constexpr double kPYTrans = 0.1;
    constexpr double kThetaDeadzone = 2.0;

    constexpr units::kilogram_t kRobotWeight = 54.43_kg;
    constexpr auto kMOI = 60;

}
