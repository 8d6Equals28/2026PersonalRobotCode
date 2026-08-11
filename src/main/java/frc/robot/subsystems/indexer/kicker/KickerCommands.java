package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.wpilibj2.command.Command;

public class KickerCommands {
    public static Command runForwardFast(KickerSubsystem kicker) {
        return kicker.run(()->{
            kicker.setSpeed(1);
        });
    }
    public static Command runForwardSlow(KickerSubsystem kicker) {
        return kicker.run(()->{
            kicker.setSpeed(0.3);
        });
    }
    public static Command runBackward(KickerSubsystem kicker) {
        return kicker.run(()->{
            kicker.setSpeed(-1);
        });
    }
    public static Command stop(KickerSubsystem kicker) {
        return kicker.run(()->{
            kicker.setSpeed(0);
        });
    }
}
