package frc.robot.subsystems.indexer.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double rotationsPerSec = 0;
    public double appliedOutput = 0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setSpeed(double speed) {}
}
