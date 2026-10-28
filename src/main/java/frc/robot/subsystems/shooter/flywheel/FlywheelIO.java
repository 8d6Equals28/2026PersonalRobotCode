package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double rotationsPerSecLeft = 0;
    public double appliedOutputLeft = 0;

    public double rotationsPerSecRight = 0;
    public double appliedOutputRight = 0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double voltageLeft, double voltageRight) {}

  public default void stop() {}
}
