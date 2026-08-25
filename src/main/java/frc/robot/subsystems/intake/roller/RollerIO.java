package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public double rotationsPerSec = 0;
    public double appliedOutput = 0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
