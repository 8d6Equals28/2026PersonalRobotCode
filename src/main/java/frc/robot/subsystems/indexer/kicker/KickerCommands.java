package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.wpilibj2.command.Command;

public class KickerCommands {
  public static Command stop(KickerSubsystem kicker) {
    return kicker.runOnce(() -> {
      kicker.stop();
    });
  }

  public static Command runForwardPID(KickerSubsystem kicker) {
    return kicker.runEnd(() -> kicker.setGoalVel(10), kicker::stop);
  }

  public static Command runBackwardPID(KickerSubsystem kicker) {
    return kicker.runEnd(() -> kicker.setGoalVel(-10), kicker::stop);
  }
}
