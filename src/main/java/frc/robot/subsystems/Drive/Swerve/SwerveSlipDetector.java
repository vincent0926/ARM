package frc.robot.subsystems.Drive.Swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Slip detection and prevention for swerve drive.
 */
public class SwerveSlipDetector {

  public static double getSlipScalingFactor(
      SwerveModuleState[] prevSetpoints, SwerveModuleState[] idealStates, double dt) {
    double maxRequiredAccel = 0.0;

    for (int i = 0; i < 4; i++) {
      double speedCurrent = prevSetpoints[i].speedMetersPerSecond;
      double speedTarget = idealStates[i].speedMetersPerSecond;
      double accel = Math.abs((speedTarget - speedCurrent) / dt);

      if (accel > maxRequiredAccel) {
        maxRequiredAccel = accel;
      }
    }

    if (maxRequiredAccel > Constants.Swerve.kMaxDriveAcceleration) {
      return Constants.Swerve.kMaxDriveAcceleration / maxRequiredAccel;
    }

    return 1.0;
  }
}
