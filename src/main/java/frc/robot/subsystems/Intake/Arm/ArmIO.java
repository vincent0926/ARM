package frc.robot.subsystems.Intake.Arm;

import edu.wpi.first.units.measure.Angle;

/**
 * Arm IO 硬體抽象介面 (IO Layer):
 * 將馬達硬體控制細節 (如 CTRE Phoenix 6 API) 與上層 Subsystem 商業邏輯解耦。
 * 便於進行模擬 (Simulation)、回放 (AdvantageKit Replay) 或更換不同廠牌馬達控制器 (如 SparkMax/REVLib)。
 */
public interface ArmIO {

    public void setPosition(Angle angle);

    public Angle getPosition();

    public void resetEncoder();

    public void configure();

}
