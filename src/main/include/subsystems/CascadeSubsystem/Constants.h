#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>

namespace CascadeConstants {
    // ports
    constexpr int kMotorPort = 12;
    constexpr int kEncoderPort = 14;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    // states
    enum CascadeStates {
      kOff,
      kPowerMode,
      kPositionMode
    };
    // default power in power mode

    constexpr double kDefaultPower = 0.0;
    // arm position constants
    constexpr units::length::meter_t kStartPosition{0.92_m};
    // constexpr units::length::meter_t kStartPosition{0.13335_m};
    // 138.30731201171875
    // arm min/max positions
    constexpr units::length::meter_t kCascadeMeterMin{kStartPosition};   // encoder Turns at the Cascade's minimum usable position
    constexpr units::length::meter_t kCascadeMeterMax{1.45_m}; // meters the Cascade is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerMeter = 329.30312383742559;
    // position deadzone
    constexpr units::length::meter_t kPositionDeadzone{3.0_cm};
    // TalonFX config
    constexpr double kP = 10.0;
    constexpr double kD = 1.4;
    constexpr double kG = 0.4;
    constexpr double kRotorToGearbox = 25.0;
}
