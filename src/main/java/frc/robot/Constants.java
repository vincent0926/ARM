package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Robot-wide constants and physical parameters.
 */
public final class Constants {

  /** Fundamental control loop period in seconds (50Hz). */
  public static final double kLooperDt = 0.02;

  /** CANivore Bus Name (Empty = RoboRIO Default) */
  public static final String kCANBusName = "9427";

  /** Drivetrain CAN Bus Name (Empty = RoboRIO Default) */
  public static final String kDriveCANBusName = "";

  /**
   * Constants specific to the Swerve Drive subsystem.
   */
  public static final class Swerve {

    // Dimensions
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    public static final double kWheelRadius = Units.inchesToMeters(2.0);

    // Module Locations (Center based)
    public static final Translation2d kFLPos = new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0);
    public static final Translation2d kFRPos = new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0);
    public static final Translation2d kBLPos = new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0);
    public static final Translation2d kBRPos = new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0);

    // Physical Limits
    public static final double kRobotMass = 44.0;
    public static final double kCoF = 0.90;
    public static final double kMaxFrictionForce = kRobotMass * 9.81 * kCoF;
    public static final double kMaxDriveAcceleration = kMaxFrictionForce / kRobotMass;
    public static final double kAccelerationSafetyFactor = 0.70;

    // Velocity Limits
    public static final double kMaxDriveVelocity = 3.5; // m/s
    public static final double kMaxAngularVelocity = 10.0; // rad/s
    public static final double kMaxTeleopAngularVelocity = 2.0; // rad/s

    // Gear Ratios
    public static enum SwerveModuleType {
      MK5N_L1(8.14, 287.0 / 11.0),
      MK5N_L2(6.75, 287.0 / 11.0),
      MK5N_L3(6.12, 287.0 / 11.0);

      public final double driveRatio;
      public final double steerRatio;

      SwerveModuleType(double drive, double steer) {
        this.driveRatio = drive;
        this.steerRatio = steer;
      }
    }

    public static final SwerveModuleType kModuleType = SwerveModuleType.MK5N_L2;
    public static final double kDriveGearRatio = kModuleType.driveRatio;
    public static final double kSteerGearRatio = kModuleType.steerRatio;

    public static final DCMotor kDriveGearbox = DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio);
    public static final DCMotor kSteerGearbox = DCMotor.getKrakenX44(1).withReduction(kSteerGearRatio);

    // CAN IDs & Magnet Offsets
    // Front Left Module (Index 0)
    public static final int kFLDriveId = 7;
    public static final int kFLSteerId = 8;
    public static final int kFLEncoderId = 4;
    public static final double kFLOffset = 0.05126953;

    // Front Right Module (Index 1)
    public static final int kFRDriveId = 5;
    public static final int kFRSteerId = 6;
    public static final int kFREncoderId = 3;
    public static final double kFROffset = 0.29174805;

    // Back Left Module (Index 2)
    public static final int kBLDriveId = 1;
    public static final int kBLSteerId = 2;
    public static final int kBLEncoderId = 1;
    public static final double kBLOffset = -0.04321289;

    // Back Right Module (Index 3)
    public static final int kBRDriveId = 3;
    public static final int kBRSteerId = 4;
    public static final int kBREncoderId = 2;
    public static final double kBROffset = -0.02539063;

    // Pigeon 2.0 IMU
    public static final int kPigeonId = 25;

    // Control Constants
    public static final class Control {
      public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
      public static final double kDrivePositionCoefficient = 1.1300236406619;

      // Drive Motor Gains
      public static final double kDrivekP = 0.0021 * kWheelCircumference;
      public static final double kDrivekI = 0.0;
      public static final double kDrivekD = 0.0;
      public static final double kDrivekS = 0.18;
      public static final double kDrivekV = 2.351 * kWheelCircumference;
      public static final double kDrivekA = 0.047 * kWheelCircumference;

      public static final double kDriveSupplyCurrentLimit = 60.0;
      public static final boolean kDriveSupplyCurrentLimitEnable = true;
      public static final double kDriveStatorCurrentLimit = 70.0;
      public static final boolean kDriveStatorCurrentLimitEnable = true;

      // Steer Motor Gains
      public static final double kSteerkP = 11.0000 * (2 * Math.PI);
      public static final double kSteerkI = 0.0000 * (2 * Math.PI);
      public static final double kSteerkD = 0.0000 * (2 * Math.PI);

      public static final double kSteerSupplyCurrentLimit = 40.0;
      public static final boolean kSteerSupplyCurrentLimitEnable = true;
      public static final double kSteerStatorCurrentLimit = 60.0;
      public static final boolean kSteerStatorCurrentLimitEnable = true;

      public static final double kClosedLoopRampPeriod = 0.02;
      public static final double kSimLoopPeriod = 0.02;
    }
  }
}
