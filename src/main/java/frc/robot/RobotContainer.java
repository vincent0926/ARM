package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);

    private final DriveSubsystem drive = DriveSubsystem.create();
    private final IntakeSubsystem intake = IntakeSubsystem.create();

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        // 預設駕駛指令：手把左搖桿控制平移 (X/Y)，右搖桿控制自轉 (Rotation)
        // 加上 Deadband 避免搖桿漂移，並依最大速度/角速度進行比例映射
        drive.setDefaultCommand(
            drive.driveCommand(
                () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.08) * Constants.Swerve.kMaxDriveVelocity,
                () -> -MathUtil.applyDeadband(controller.getLeftX(), 0.08) * Constants.Swerve.kMaxDriveVelocity,
                () -> -MathUtil.applyDeadband(controller.getRightX(), 0.08) * Constants.Swerve.kMaxTeleopAngularVelocity,
                () -> true // Field-relative (場地導向)
            )
        );
    }

    private void configureBindings() {
        /*
         * 按鈕綁定配置：
         * 1. A 鍵 (按住): intakeRunCommand (手臂放下並持續吸入)
         * 2. B 鍵 (單次): intakeUpCommand (手臂抬起)
         * 3. Back 鍵 (單次): zeroGyroCommand (陀螺儀校正歸零)
         * 4. X 鍵 (按住): xLockCommand (輪胎打 X 煞車鎖定)
         */
        controller.a().whileTrue(intake.intakeRunCommand());
        controller.b().onTrue(intake.intakeUpCommand());

        controller.back().onTrue(drive.zeroGyroCommand());
        controller.x().whileTrue(drive.xLockCommand());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}