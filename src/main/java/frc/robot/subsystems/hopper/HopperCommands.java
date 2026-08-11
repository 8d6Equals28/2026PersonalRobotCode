package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommands {
    public static Command runForwardFast(HopperSubsystem hopper) {
        return hopper.run(()->{
            hopper.setVoltage(1);
        });
    }
    public static Command runForwardSlow(HopperSubsystem hopper) {
        return hopper.run(()->{
            hopper.setVoltage(0.3);
        });
    }
    public static Command runBackward(HopperSubsystem hopper) {
        return hopper.run(()->{
            hopper.setVoltage(-1);
        });
    }
}
