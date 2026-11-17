package frc.robot.subsystems.intake.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmCommands {
  public static Command stop(ArmSubsystem arm) {
    return arm.runOnce(() -> {
      arm.stop();
    });
  }

  public static Command setPositionUp(ArmSubsystem arm) {
    return arm.runEnd(() -> arm.setGoalPosition(30), arm::stop);
  }

  public static Command setPositionDown(ArmSubsystem arm) {
    return arm.runEnd(() -> arm.setGoalPosition(10), arm::stop);
  }

  public static Command setPositionZero(ArmSubsystem arm) {
    return arm.runEnd(() -> arm.setGoalPosition(0), arm::stop);
  }
}
