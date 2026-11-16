package frc.robot.subsystems.intake.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double rotationsPerSec = 0;
    public double appliedOutput = 0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(Double voltage) {}

  public default void stop() {}

  public default void resetPivotPosition(Double position) {}
}
