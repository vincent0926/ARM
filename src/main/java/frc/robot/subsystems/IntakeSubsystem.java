package frc.robot.subsystems;

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

    public Command intakeDownCommand() {
        return this.runOnce(this::intakeDown);
    }

    public Command intakeUpCommand() {
        return this.runOnce(this::intakeUp);
    }

    /** 持續吸入,直到這個 command 被取消(例如放開按鈕)才停止滾輪 */
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

    /** 手臂放下 + 滾輪吸入,適合綁在「按住吸取」的按鈕上 */
    public Command intakeRunCommand() {
        return Commands.sequence(this.intakeDownCommand(), this.intakeRollRunCommand());
    }

    public Command stopIntakeCommand() {
        return this.runOnce(() -> {
            this.roller.stop();
            setState(IntakeState.NONE);
        });
    }

    // public Command intakeUpAndShootCommand() {
    // return Commands.parallel(
    // this.intakeUpCommand(),
    // // 包成同一個 command 內部呼叫,避免外部亂呼叫造成手臂/滾輪不同步
    // Commands.runOnce(() -> {
    // this.roller.setVoltage(Volts.of(SCORE_VOLTAGE));
    // setState(IntakeState.SCORING);
    // }, this));
    // }

    public Command intakeDownAndStopCommand() {
        return Commands.parallel(
                this.intakeDownCommand(),
                Commands.runOnce(() -> {
                    this.roller.stop();
                    setState(IntakeState.NONE);
                }, this));
    }

}