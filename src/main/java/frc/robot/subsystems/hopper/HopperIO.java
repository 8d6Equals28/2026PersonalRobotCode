package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        double rotationsPerSec;
        double appliedOutput;
    }

    public default void updateInputs(HopperIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
