
package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class ArmIOHardware implements ArmIO {

        private static final Angle start_position = Degrees.of(150);

        private static final Angle minangle = Degrees.of(0);
        private static final Angle maxangle = Degrees.of(130);

        private final TalonFX armMotor;
        private final TalonFX followMotor;

        private final StatusSignal<Angle> armPosition;

        private final MotionMagicVoltage armOutput = new MotionMagicVoltage(Degrees.of(0));

        public ArmIOHardware() {

                armMotor = new TalonFX(9, "9427");
                followMotor = new TalonFX(10, "9427");

                armPosition = armMotor.getPosition();

                configure();

                resetEncoder();
        }

        @Override
        public void setPosition(Angle angle) {

                armMotor.setControl(
                                armOutput.withPosition(angle));
        }

        @Override
        public Angle getPosition() {

                armPosition.refresh();

                return armPosition.getValue();
        }

        @Override
        public void resetEncoder() {

                armMotor.getConfigurator().setPosition(
                                start_position);

                followMotor.getConfigurator().setPosition(
                                start_position);

        }

        @Override
        public void configure() {

                TalonFXConfiguration armConfig = new TalonFXConfiguration();

                armConfig.CurrentLimits
                                .withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true);

                armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                armConfig.Feedback.SensorToMechanismRatio = 10.0;

                armConfig.Slot0.kP = 6.128;
                armConfig.Slot0.kI = 0.0;
                armConfig.Slot0.kD = 0.7965;

                armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

                armConfig.Slot0.kG = 0.0;

                armConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;

                armConfig.MotionMagic.MotionMagicAcceleration = 2.0;

                armConfig.MotionMagic.MotionMagicJerk = 10.0;

                armConfig.SoftwareLimitSwitch
                                .withReverseSoftLimitThreshold(
                                                minangle.in(Degrees) / 360.0)
                                .withReverseSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(
                                                maxangle.in(Degrees) / 360.0)
                                .withForwardSoftLimitEnable(true);

                armMotor.getConfigurator().apply(armConfig);

                followMotor.getConfigurator().apply(armConfig);

                followMotor.setControl(
                                new Follower(
                                                armMotor.getDeviceID(),
                                                MotorAlignmentValue.Opposed));
        }
}
