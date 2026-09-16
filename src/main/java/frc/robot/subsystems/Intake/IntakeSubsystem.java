package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Arm.ArmIOHardware;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOHardware;

public class IntakeSubsystem extends SubsystemBase {

    // TODO(上機測試確認): 吸入/送料的電壓大小與正負號需要實際測試調整
    private static final double INTAKE_VOLTAGE = 5.5;
    // private static final double SCORE_VOLTAGE = -5.5;

    private final ArmIO arm;
    private final RollerIO roller;

    public enum IntakeState {
        NONE, // 閒置:手臂收起、滾輪停止
        INTAKING, // 手臂放下、滾輪吸入中
        // SCORING // 手臂抬起、滾輪反轉送出燃料
    }

    private IntakeState state = IntakeState.NONE;

    public IntakeSubsystem(ArmIO arm, RollerIO roller) {
        this.arm = arm;
        this.roller = roller;
    }

    public static IntakeSubsystem create() {
        return new IntakeSubsystem(
                new ArmIOHardware(),
                new RollerIOHardware());
    }

    private void setState(IntakeState newState) {
        this.state = newState;
    }

    public IntakeState getState() {
        return this.state;
    }

    public void intakeDown() {
        this.arm.setPosition(Degrees.of(2));
    }

    public void intakeUp() {
        this.arm.setPosition(Degrees.of(129));
    }

    /** 手臂放下指令：發送目標角度給 Arm 馬達控制器 */
    public Command intakeDownCommand() {
        return this.runOnce(this::intakeDown);
    }

    /** 手臂抬起指令：發送目標角度給 Arm 馬達控制器 */
    public Command intakeUpCommand() {
        return this.runOnce(this::intakeUp);
    }

    /**
     * 持續吸入指令：
     * - runEnd 結構：
     *   1. 啟動階段 (Execute): 每個週期輸出電壓給滾輪馬達，並更新狀態為 INTAKING。
     *   2. 結束階段 (End/Interrupt): 當指令被中斷（例如按鈕放開）時，自動執行停止滾輪並切換狀態為 NONE。
     * - Subsystem Requirement (傳入 this): 宣告此指令獨佔 IntakeSubsystem，防止其他吸取指令同時競爭造成邏輯衝突。
     */
    public Command intakeRollRunCommand() {
        return Commands.runEnd(
                () -> {
                    this.roller.setVoltage(Volts.of(INTAKE_VOLTAGE));
                    setState(IntakeState.INTAKING);
                },
                () -> {
                    this.roller.stop();
                    setState(IntakeState.NONE);
                },
                this);
    }

    /**
     * 組合指令 (Sequence)：
     * 依序執行「手臂放下 (runOnce)」->「持續吸入 (runEnd)」。
     * 適用於綁定在搖桿的按住按鈕 (whileTrue) 上。
     */
    public Command intakeRunCommand() {
        return Commands.sequence(this.intakeDownCommand(), this.intakeRollRunCommand());
    }

    /** 停止吸取指令：立即關閉滾輪並復位狀態 */
    public Command stopIntakeCommand() {
        return this.runOnce(() -> {
            this.roller.stop();
            setState(IntakeState.NONE);
        });
    }

    /** 手臂放下並停止滾輪 (Parallel)：同時執行放下動作並將滾輪斷電 */
    public Command intakeDownAndStopCommand() {
        return Commands.parallel(
                this.intakeDownCommand(),
                Commands.runOnce(() -> {
                    this.roller.stop();
                    setState(IntakeState.NONE);
                }, this));
    }

}