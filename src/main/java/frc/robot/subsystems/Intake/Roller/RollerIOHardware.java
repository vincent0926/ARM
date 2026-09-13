package frc.robot.subsystems.Intake.Roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;

public class RollerIOHardware implements RollerIO {
    private final TalonFX roller = new TalonFX(11, "9427");
    private final TalonFX rollerFollower = new TalonFX(12, "9427");

    // control request 物件重複使用,避免每次呼叫都 new,減少 GC 壓力
    private final VoltageOut rollVoltageOut = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Voltage> motorVoltageSignal = roller.getMotorVoltage();

    private double targetRollerVoltage = 0.0;

    public RollerIOHardware() {
        configureMotor();

        rollerFollower.setControl(new Follower(roller.getDeviceID(), MotorAlignmentValue.Opposed));

        motorVoltageSignal.setUpdateFrequency(50);

        roller.optimizeBusUtilization();
        rollerFollower.optimizeBusUtilization();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true);

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        roller.getConfigurator().apply(config);
        rollerFollower.getConfigurator().apply(config);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        targetRollerVoltage = voltage.in(Volts);
        roller.setControl(rollVoltageOut.withOutput(voltage));
    }

    @Override
    public Voltage getVoltage() {
        motorVoltageSignal.refresh();
        return motorVoltageSignal.getValue();
    }

    @Override
    public void stop() {
        targetRollerVoltage = 0.0;
        roller.setControl(rollVoltageOut.withOutput(Volts.of(0)));
    }
}