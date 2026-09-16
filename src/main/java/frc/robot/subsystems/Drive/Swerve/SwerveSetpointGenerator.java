package frc.robot.subsystems.Drive.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Kinematic Feasibility Filter for swerve drive.
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics mKinematics;

  private SwerveModuleState[] mPrevSetpoint;
  private ChassisSpeeds mPrevChassisSpeeds = new ChassisSpeeds();
  private final double[] mFilteredVoltages = new double[] {12.0, 12.0, 12.0, 12.0};

  public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
    mKinematics = kinematics;
    mPrevSetpoint = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
    }
  }

  private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public SwerveModuleState[] generateSetpoint(ChassisSpeeds desiredSpeeds, double[] driveVoltages) {
    for (int i = 0; i < 4; i++) {
      mFilteredVoltages[i] = mFilteredVoltages[i] * 0.85 + driveVoltages[i] * 0.15;
    }

    ChassisSpeeds feasibleSpeeds =
        enforceFeasibility(desiredSpeeds, Constants.kLooperDt, mFilteredVoltages);

    SwerveModuleState[] feasibleStates = mKinematics.toSwerveModuleStates(feasibleSpeeds);

    boolean isStationary =
        Math.abs(feasibleSpeeds.vxMetersPerSecond) < 0.01
            && Math.abs(feasibleSpeeds.vyMetersPerSecond) < 0.01
            && Math.abs(feasibleSpeeds.omegaRadiansPerSecond) < 0.01;

    for (int i = 0; i < 4; i++) {
      if (isStationary) {
        feasibleStates[i].speedMetersPerSecond = 0.0;
        feasibleStates[i].angle = mPrevSetpoint[i].angle;
      }
      feasibleStates[i] = optimize(feasibleStates[i], mPrevSetpoint[i].angle);
    }

    mPrevSetpoint = feasibleStates;
    mPrevChassisSpeeds = feasibleSpeeds;
    return feasibleStates;
  }

  private ChassisSpeeds enforceFeasibility(
      ChassisSpeeds desiredSpeeds, double dt, double[] driveVoltages) {

    SwerveModuleState[] idealStates = mKinematics.toSwerveModuleStates(desiredSpeeds);

    for (int i = 0; i < 4; i++) {
      idealStates[i] = optimize(idealStates[i], mPrevSetpoint[i].angle);
    }

    double limitFriction =
        Constants.Swerve.kMaxDriveAcceleration * Constants.Swerve.kAccelerationSafetyFactor;

    double[] linearAccelerations = new double[4];
    double[] maxAllowedLinearAccelerations = new double[4];

    for (int i = 0; i < 4; i++) {
      double vCurrent = mPrevSetpoint[i].speedMetersPerSecond;
      double vTarget = idealStates[i].speedMetersPerSecond;

      double motorSpeedRadPerSec =
          (Math.abs(vCurrent) / Constants.Swerve.Control.kWheelCircumference)
              * Constants.Swerve.kDriveGearRatio
              * (2 * Math.PI);

      double torqueCurrentLimit =
          Constants.Swerve.Control.kDriveStatorCurrentLimit
              * Constants.Swerve.kDriveGearbox.KtNMPerAmp;

      double validVoltage = Math.max(0.0, driveVoltages[i]);
      double currentAtVoltage =
          Constants.Swerve.kDriveGearbox.getCurrent(motorSpeedRadPerSec, validVoltage);
      double torqueVoltageLimit = currentAtVoltage * Constants.Swerve.kDriveGearbox.KtNMPerAmp;

      double maxTorqueBot = Math.min(torqueCurrentLimit, Math.max(0.0, torqueVoltageLimit));

      double maxWheelTorque = maxTorqueBot * Constants.Swerve.kDriveGearRatio;
      double maxForceOneWheel = maxWheelTorque / Constants.Swerve.kWheelRadius;
      double maxMotorAccel = (4.0 * maxForceOneWheel) / Constants.Swerve.kRobotMass;

      double aMax = Math.min(limitFriction, maxMotorAccel);

      double aLin = (vTarget - vCurrent) / dt;
      linearAccelerations[i] = aLin;
      maxAllowedLinearAccelerations[i] = aMax;
    }

    double minK = 1.0;
    for (int i = 0; i < 4; i++) {
      double req = Math.abs(linearAccelerations[i]);
      double allowed = maxAllowedLinearAccelerations[i];

      if (req > allowed + 1e-6) {
        double k = allowed / req;
        if (k < minK) {
          minK = k;
        }
      }
    }

    double deltaVx = desiredSpeeds.vxMetersPerSecond - mPrevChassisSpeeds.vxMetersPerSecond;
    double deltaVy = desiredSpeeds.vyMetersPerSecond - mPrevChassisSpeeds.vyMetersPerSecond;
    double deltaOmega =
        desiredSpeeds.omegaRadiansPerSecond - mPrevChassisSpeeds.omegaRadiansPerSecond;

    double feasibleVx = mPrevChassisSpeeds.vxMetersPerSecond + deltaVx * minK;
    double feasibleVy = mPrevChassisSpeeds.vyMetersPerSecond + deltaVy * minK;
    double feasibleOmega = mPrevChassisSpeeds.omegaRadiansPerSecond + deltaOmega * minK;

    return new ChassisSpeeds(feasibleVx, feasibleVy, feasibleOmega);
  }

  public void reset() {
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
      mFilteredVoltages[i] = 12.0;
    }
    mPrevChassisSpeeds = new ChassisSpeeds();
  }
}
