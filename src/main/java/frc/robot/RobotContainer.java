package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class RobotContainer {
    // 1. 使用 CommandXboxController 取代舊版 XboxController，才能直接調用 .a()、.b() 與 .whileTrue()
    private final CommandXboxController controller = new CommandXboxController(0);

    private final IntakeSubsystem intake = IntakeSubsystem.create();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        /*
         * 按鈕綁定配置：
         * 1. whileTrue (按住觸發/放開中斷):
         *    - 按下 A 鍵時排程 intakeRunCommand (手臂放下並持續吸入)。
         *    - 放開 A 鍵時強制中斷 (Cancel) 該指令，觸發 intakeRollRunCommand 內 runEnd 的清理區塊以停止滾輪。
         * 2. onTrue (邊緣觸發/單次執行):
         *    - 按下 B 鍵瞬間觸發 intakeUpCommand (執行一次手臂抬起)。
         */
        controller.a().whileTrue(intake.intakeRunCommand());
        controller.b().onTrue(intake.intakeUpCommand());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}