package frc.robot.subsystems.Drive.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.SwerveModuleType;
import frc.robot.util.Phoenix6Util;

/**
 * Phoenix 6 configuration factory for swerve modules.
 */
public class SwerveModuleConfigurator {

  public static boolean configure(
      TalonFX driveMotor,
      TalonFX steerMotor,
      CANcoder encoder,
      SwerveModuleType moduleType,
      double steerOffsetRotations) {
    boolean allOk = true;

    // --- CANcoder Configuration ---
    var coderConfig = new CANcoderConfiguration();
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    coderConfig.MagnetSensor.MagnetOffset = steerOffsetRotations;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> encoder.getConfigurator().apply(coderConfig), "Module Encoder Config");

    Timer.delay(0.1);

    // --- Drive Motor Configuration ---
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveConfig.Slot0.kP = Constants.Swerve.Control.kDrivekP;
    driveConfig.Slot0.kI = Constants.Swerve.Control.kDrivekI;
    driveConfig.Slot0.kD = Constants.Swerve.Control.kDrivekD;
    driveConfig.Slot0.kS = Constants.Swerve.Control.kDrivekS;
    driveConfig.Slot0.kV = Constants.Swerve.Control.kDrivekV;
    driveConfig.Slot0.kA = Constants.Swerve.Control.kDrivekA;

    driveConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kDriveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kDriveSupplyCurrentLimitEnable;
    driveConfig.CurrentLimits.StatorCurrentLimit =
        Constants.Swerve.Control.kDriveStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Swerve.Control.kDriveStatorCurrentLimitEnable;

    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kDriveGearRatio;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> driveMotor.getConfigurator().apply(driveConfig), "Module Drive Motor Config");

    Timer.delay(0.1);

    // --- Steer Motor Configuration ---
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    steerConfig.Slot0.kP = Constants.Swerve.Control.kSteerkP;
    steerConfig.Slot0.kI = Constants.Swerve.Control.kSteerkI;
    steerConfig.Slot0.kD = Constants.Swerve.Control.kSteerkD;

    steerConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kSteerSupplyCurrentLimit;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kSteerSupplyCurrentLimitEnable;
    steerConfig.CurrentLimits.StatorCurrentLimit =
        Constants.Swerve.Control.kSteerStatorCurrentLimit;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Swerve.Control.kSteerStatorCurrentLimitEnable;

    steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.kSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> steerMotor.getConfigurator().apply(steerConfig), "Module Steer Motor Config");

    return allOk;
  }
}
