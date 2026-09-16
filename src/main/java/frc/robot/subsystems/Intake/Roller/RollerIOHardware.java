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

    // 重複使用 VoltageOut 控制物件，避免每次週期呼叫時重複實例化產生 GC 垃圾回收延遲
    // withEnableFOC(true) 啟用磁場導向控制 (Field-Oriented Control)，提升馬達扭矩響應與運轉效率 (需 Pro 授權)
    private final VoltageOut rollVoltageOut = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Voltage> motorVoltageSignal = roller.getMotorVoltage();

    private double targetRollerVoltage = 0.0;

    public RollerIOHardware() {
        configureMotor();

        // 從動馬達反向對齊：雙馬達對接滾軸時轉向相反，需設為 Opposed
        rollerFollower.setControl(new Follower(roller.getDeviceID(), MotorAlignmentValue.Opposed));

        motorVoltageSignal.setUpdateFrequency(50);

        // 匯流排使用率優化：關閉或降低不需要的訊號頻率，釋放 CAN Bus 頻寬
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