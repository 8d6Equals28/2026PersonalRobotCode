package frc.robot.subsystems.drivetrain.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPosition = 0;
    public double[] yawTimestamps = new double[] {};
    public double[] yawPositions = new double[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
