package frc.robot.subsystems.indexer.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public double rotationsPerSec = 0;
        public double appliedOutput = 0;
    }

    public default void updateInputs(HopperIOInputs inputs) {}

    public default void setSpeed(double speed) {}
}
