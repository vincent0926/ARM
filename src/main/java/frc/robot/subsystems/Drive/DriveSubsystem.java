package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve.SwerveModule;
import frc.robot.subsystems.Drive.Swerve.SwerveSetpointGenerator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Swerve Drive Subsystem using IO Architecture and WPILib Command-based framework.
 */
public class DriveSubsystem extends SubsystemBase {

  private final DriveIO mIO;
  private final DriveIO.DriveIOInputs mInputs = new DriveIO.DriveIOInputs();

  private final SwerveModule[] mModules;
  private final SwerveDriveKinematics mKinematics;
  private final SwerveSetpointGenerator mSetpointGenerator;
  private final SwerveDriveOdometry mOdometry;

  private final SwerveModulePosition[] mModulePositions;

  private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
  private boolean mIsXLocked = false;
  private boolean mIsOpenLoop = false;

  public DriveSubsystem(DriveIO io) {
    this.mIO = io;

    this.mModules =
        new SwerveModule[] {
          new SwerveModule(0),
          new SwerveModule(1),
          new SwerveModule(2),
          new SwerveModule(3)
        };

    this.mKinematics =
        new SwerveDriveKinematics(
            Constants.Swerve.kFLPos,
            Constants.Swerve.kFRPos,
            Constants.Swerve.kBLPos,
            Constants.Swerve.kBRPos);

    this.mSetpointGenerator = new SwerveSetpointGenerator(mKinematics);

    this.mModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    try {
      Thread.sleep(250);
    } catch (InterruptedException ignored) {
    }

    mIO.updateInputs(mInputs);

    for (int i = 0; i < 4; i++) {
      mModules[i].updatePosition(
          mInputs.drivePositionRotations[i],
          mInputs.steerPositionRotations[i],
          mModulePositions[i]);
    }

    this.mOdometry =
        new SwerveDriveOdometry(
            mKinematics, mInputs.gyroYaw, mModulePositions, new Pose2d());

    mIO.startOdometryThread();
  }

  public static DriveSubsystem create() {
    return new DriveSubsystem(new DriveIOHardware());
  }

  @Override
  public void periodic() {
    mIO.updateInputs(mInputs);

    for (int i = 0; i < 4; i++) {
      mModules[i].updatePosition(
          mInputs.drivePositionRotations[i],
          mInputs.steerPositionRotations[i],
          mModulePositions[i]);
    }

    mOdometry.update(mInputs.gyroYaw, mModulePositions);

    if (mIsXLocked) {
      applyXLock();
      return;
    }

    handleVelocityControl();
    outputTelemetry();
  }

  private void handleVelocityControl() {
    boolean isZero =
        Math.abs(mDesiredChassisSpeeds.vxMetersPerSecond) < 1e-3
            && Math.abs(mDesiredChassisSpeeds.vyMetersPerSecond) < 1e-3
            && Math.abs(mDesiredChassisSpeeds.omegaRadiansPerSecond) < 1e-3;

    if (isZero) {
      for (int i = 0; i < 4; i++) {
        mIO.setDriveVoltage(i, 0.0);
      }
      return;
    }

    SwerveModuleState[] setpointStates =
        mSetpointGenerator.generateSetpoint(mDesiredChassisSpeeds, mInputs.driveSupplyVoltage);

    for (int i = 0; i < 4; i++) {
      SwerveModule.ModuleIO ioCmd =
          mModules[i].updateSetpoint(
              setpointStates[i], mInputs.steerPositionRotations[i], mIsOpenLoop);

      if (ioCmd.isOpenLoop) {
        mIO.setDriveVoltage(i, ioCmd.driveDemand);
      } else {
        mIO.setDriveVelocity(i, ioCmd.driveDemand);
      }
      mIO.setSteerPosition(i, ioCmd.steerDemand);
    }
  }

  private void applyXLock() {
    // Front Left: 45 deg, Front Right: -45 deg, Back Left: -45 deg, Back Right: 45 deg
    mIO.setDriveVoltage(0, 0);
    mIO.setDriveVoltage(1, 0);
    mIO.setDriveVoltage(2, 0);
    mIO.setDriveVoltage(3, 0);

    mIO.setSteerPosition(0, 45.0 / 360.0);
    mIO.setSteerPosition(1, -45.0 / 360.0);
    mIO.setSteerPosition(2, -45.0 / 360.0);
    mIO.setSteerPosition(3, 45.0 / 360.0);
  }

  public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.mIsXLocked = false;
    this.mIsOpenLoop = isOpenLoop;
    this.mDesiredChassisSpeeds = speeds;
  }

  public void drive(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec, boolean fieldRelative) {
    this.mIsXLocked = false;
    ChassisSpeeds speeds;
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, mInputs.gyroYaw);
    } else {
      speeds = new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
    }
    this.mDesiredChassisSpeeds = ChassisSpeeds.discretize(speeds, Constants.kLooperDt);
  }

  public void stop() {
    this.mIsXLocked = false;
    this.mDesiredChassisSpeeds = new ChassisSpeeds();
    for (int i = 0; i < 4; i++) {
      mIO.setDriveVoltage(i, 0.0);
    }
  }

  public void setXLock(boolean locked) {
    this.mIsXLocked = locked;
  }

  public void setGyroYaw(double degrees) {
    mIO.setGyroYaw(degrees);
  }

  public void resetPose(Pose2d pose) {
    mOdometry.resetPosition(mInputs.gyroYaw, mModulePositions, pose);
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public Rotation2d getGyroYaw() {
    return mInputs.gyroYaw;
  }

  public SwerveDriveKinematics getKinematics() {
    return mKinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return mKinematics.toChassisSpeeds(
        mModules[0].getState(mInputs.driveVelocityRotationsPerSec[0], mInputs.steerPositionRotations[0]),
        mModules[1].getState(mInputs.driveVelocityRotationsPerSec[1], mInputs.steerPositionRotations[1]),
        mModules[2].getState(mInputs.driveVelocityRotationsPerSec[2], mInputs.steerPositionRotations[2]),
        mModules[3].getState(mInputs.driveVelocityRotationsPerSec[3], mInputs.steerPositionRotations[3])
    );
  }

  // --- Commands ---

  public Command driveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    return run(() -> {
      double vx = xSupplier.getAsDouble();
      double vy = ySupplier.getAsDouble();
      double rot = rotSupplier.getAsDouble();
      boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();
      drive(vx, vy, rot, fieldRelative);
    });
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command xLockCommand() {
    return runOnce(() -> setXLock(true));
  }

  public Command zeroGyroCommand() {
    return runOnce(() -> setGyroYaw(0.0));
  }

  private void outputTelemetry() {
    SmartDashboard.putNumber("Drive/GyroYawDeg", mInputs.gyroYaw.getDegrees());
    SmartDashboard.putNumber("Drive/PoseX", getPose().getX());
    SmartDashboard.putNumber("Drive/PoseY", getPose().getY());
    SmartDashboard.putNumber("Drive/PoseHeadingDeg", getPose().getRotation().getDegrees());
  }
}
