package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;

public class RollerCommands {
  public static Command runForwardFast(RollerSubsystem roller) {
    return roller.run(() -> {
      roller.setSpeed(1);
    });
  }

  public static Command runForwardSlow(RollerSubsystem roller) {
    return roller.run(() -> {
      roller.setSpeed(0.3);
    });
  }

  public static Command runBackward(RollerSubsystem roller) {
    return roller.run(() -> {
      roller.setSpeed(-1);
    });
  }

  public static Command stop(RollerSubsystem roller) {
    return roller.run(() -> {
      roller.setSpeed(0);
    });
  }
}
