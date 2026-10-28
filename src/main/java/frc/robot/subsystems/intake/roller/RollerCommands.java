package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;

public class RollerCommands {
  public static Command stop(RollerSubsystem roller) {
    return roller.runOnce(() -> {
      roller.stop();
    });
  }

  public static Command runForwardPID(RollerSubsystem roller) {
    return roller.runEnd(() -> roller.setGoalVel(10), roller::stop);
  }

  public static Command runBackwardPID(RollerSubsystem roller) {
    return roller.runEnd(() -> roller.setGoalVel(-10), roller::stop);
  }
}
