package frc.robot.subsystems.Drive.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Pure-logic swerve module controller.
 */
public class SwerveModule {

  /**
   * Output container for motor commands.
   */
  public static class ModuleIO {
    /** Drive motor command: velocity (rot/s) in closed-loop, or voltage in open-loop. */
    public double driveDemand;

    /** Steer motor command: target angle in rotations. */
    public double steerDemand;

    /** True if using open-loop (voltage) control for drive. */
    public boolean isOpenLoop = false;
  }

  private final int mModuleIndex;
  private final ModuleIO mModuleIO = new ModuleIO();

  public SwerveModule(int index) {
    mModuleIndex = index;
  }

  public ModuleIO updateSetpoint(
      SwerveModuleState desiredState, double currentSteerAngleRotations, boolean isOpenLoop) {

    Rotation2d currentAngle = Rotation2d.fromRotations(currentSteerAngleRotations);

    // Cosine scaling: reduce commanded speed based on angular error
    Rotation2d error = desiredState.angle.minus(currentAngle);
    double cosineScalar = error.getCos();
    double scaledSpeed = desiredState.speedMetersPerSecond * Math.abs(cosineScalar);

    // Convert to motor commands
    mModuleIO.isOpenLoop = isOpenLoop;
    if (isOpenLoop) {
      mModuleIO.driveDemand = (scaledSpeed / Constants.Swerve.kMaxDriveVelocity) * 12.0;
    } else {
      double velocityRotPerSec = scaledSpeed / Constants.Swerve.Control.kWheelCircumference;
      mModuleIO.driveDemand = velocityRotPerSec;
    }

    mModuleIO.steerDemand = desiredState.angle.getRotations();

    return mModuleIO;
  }

  public SwerveModuleState getState(double driveVelocityRotPerSec, double steerPositionRotations) {
    double speedMetersPerSecond =
        driveVelocityRotPerSec * Constants.Swerve.Control.kWheelCircumference;
    return new SwerveModuleState(
        speedMetersPerSecond, Rotation2d.fromRotations(steerPositionRotations));
  }

  public SwerveModulePosition getPosition(
      double drivePositionRotations, double steerPositionRotations) {
    double distanceMeters =
        drivePositionRotations
            * Constants.Swerve.Control.kWheelCircumference
            * Constants.Swerve.Control.kDrivePositionCoefficient;
    return new SwerveModulePosition(
        distanceMeters, Rotation2d.fromRotations(steerPositionRotations));
  }

  public void updatePosition(
      double drivePositionRotations, double steerPositionRotations, SwerveModulePosition out) {
    double distanceMeters = drivePositionRotations * Constants.Swerve.Control.kWheelCircumference;
    out.distanceMeters = distanceMeters * Constants.Swerve.Control.kDrivePositionCoefficient;
    out.angle = Rotation2d.fromRotations(steerPositionRotations);
  }
}
