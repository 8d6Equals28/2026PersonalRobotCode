package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelCommands {
  public static Command stop(FlywheelSubsystem flywheel) {
    return flywheel.runOnce(() -> {
      flywheel.stop();
    });
  }

  public static Command runForwardPID(FlywheelSubsystem flywheel) {
    return flywheel.runEnd(() -> flywheel.setGoalVel(60), flywheel::stop);
  }

  public static Command runBackwardPID(FlywheelSubsystem flywheel) {
    return flywheel.runEnd(() -> flywheel.setGoalVel(-10), flywheel::stop);
  }
}
