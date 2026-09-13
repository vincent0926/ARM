package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final CommandXboxController controller = new CommandXboxController(0);

    private final IntakeSubsystem intake = IntakeSubsystem.create();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // 按住 A:手臂放下 + 滾輪吸入,放開:滾輪停止(手臂維持放下)
        controller.a().whileTrue(intake.intakeRunCommand());

        // 按一下 B:手臂抬起(滾輪狀態不受影響)
        controller.b().onTrue(intake.intakeUpCommand());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}