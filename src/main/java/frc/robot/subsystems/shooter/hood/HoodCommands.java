package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;

public class HoodCommands {
  public static Command stop(HoodSubsystem hood) {
    return hood.runOnce(() -> {
      hood.stop();
    });
  }

  public static Command setPositionUp(HoodSubsystem hood) {
    return hood.runEnd(() -> hood.setGoalPosition(30), hood::stop);
  }

  public static Command setPositionDown(HoodSubsystem hood) {
    return hood.runEnd(() -> hood.setGoalPosition(10), hood::stop);
  }

  public static Command setPositionZero(HoodSubsystem hood) {
    return hood.runEnd(() -> hood.setGoalPosition(0), hood::stop);
  }
}
