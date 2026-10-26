package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommands {
  public static Command stop(HopperSubsystem hopper) {
    return hopper.runOnce(() -> {
      hopper.stop();
    });
  }

  public static Command runForwardPID(HopperSubsystem hopper) {
    return hopper.runEnd(() -> hopper.setGoalVel(10), hopper::stop);
  }

  public static Command runBackwardPID(HopperSubsystem hopper) {
    return hopper.runEnd(() -> hopper.setGoalVel(-10), hopper::stop);
  }
}
