package frc.robot.subsystems.intake.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double position = 0;
    public double appliedOutput = 0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}
}
