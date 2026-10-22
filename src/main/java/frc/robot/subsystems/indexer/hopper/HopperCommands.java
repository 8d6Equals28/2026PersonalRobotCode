package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommands {
  public static Command runForwardFast(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setSpeed(1);
    });
  }

  public static Command runForwardSlow(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setSpeed(0.3);
    });
  }

  public static Command runBackward(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setSpeed(-1);
    });
  }

  public static Command stop(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setSpeed(0);
    });
  }

  public static Command runForwardPID(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setGoalVel(16);
    });
  }

  public static Command runBackwardPID(HopperSubsystem hopper) {
    return hopper.run(() -> {
      hopper.setGoalVel(-16);
    });
  }
}
