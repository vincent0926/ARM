package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.units.measure.Voltage;

public interface RollerIO {
    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public void stop();
}