package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Hardware abstraction interface for the Drive subsystem.
 *
 * <p>Module Indexing Convention:
 * <ul>
 *   <li>0 = Front Left (FL)</li>
 *   <li>1 = Front Right (FR)</li>
 *   <li>2 = Back Left (BL)</li>
 *   <li>3 = Back Right (BR)</li>
 * </ul>
 */
public interface DriveIO {

  /**
   * Container for all drive subsystem sensor inputs.
   */
  public static class DriveIOInputs {
    public double timestamp;

    public Rotation2d gyroYaw = new Rotation2d();
    public double gyroYawVelocityRadPerSec;
    public double[] accelMetersPerSec2 = new double[3];

    public double gyroPitchVelocityRadPerSec;
    public double gyroRollVelocityRadPerSec;
    public double gyroPitchRad;
    public double gyroRollRad;

    public double[] drivePositionRotations = new double[4];
    public double[] driveVelocityRotationsPerSec = new double[4];
    public double[] driveAppliedVolts = new double[4];
    public double[] driveCurrentAmps = new double[4];
    public double[] driveSupplyVoltage = new double[4];

    public double[] steerPositionRotations = new double[4];
    public double[] steerVelocityRotationsPerSec = new double[4];
    public double[] steerAppliedVolts = new double[4];
    public double[] steerCurrentAmps = new double[4];

    public double[] steerAbsolutePositionRotations = new double[4];

    public boolean[] driveMotorConnected = new boolean[] {true, true, true, true};
    public boolean[] steerMotorConnected = new boolean[] {true, true, true, true};
    public boolean[] cancoderConnected = new boolean[] {true, true, true, true};
    public boolean pigeonConnected = true;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setDriveVoltage(int moduleIndex, double volts) {}

  public default void setSteerVoltage(int moduleIndex, double volts) {}

  public default void setDriveVelocity(int moduleIndex, double velocityRadPerSec) {}

  public default void setSteerPosition(int moduleIndex, double rotations) {}

  public default void setDriveBrakeMode(boolean enable) {}

  public default void setGyroYaw(double degrees) {}

  public default double[] getAbsolutePositionsRotations() {
    return new double[4];
  }

  public default double[] getCurrentMagnetOffsets() {
    return new double[4];
  }

  public default void setMagnetOffset(int moduleIndex, double offset) {}

  public default void playTone(double frequencyHz) {}

  public default int[] getDriveMotorFirmwareVersions() {
    return new int[4];
  }

  public default int[] getSteerMotorFirmwareVersions() {
    return new int[4];
  }

  public default int getPigeonFirmwareVersion() {
    return 0;
  }

  /**
   * Thread-safe snapshot of odometry-critical sensor data.
   */
  public static class OdometrySnapshot {
    public double timestamp;
    public Rotation2d gyroYaw = new Rotation2d();
    public double gyroYawVelocityRadPerSec;
    public double gyroPitchVelocityRadPerSec;
    public double gyroRollVelocityRadPerSec;
    public double[] accelMetersPerSec2 = new double[3];
    public double[] drivePositionRotations = new double[4];
    public double[] steerPositionRotations = new double[4];
    public SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
  }

  public default OdometrySnapshot getLatestOdometrySnapshot() {
    return new OdometrySnapshot();
  }

  public default void startOdometryThread() {}
}
