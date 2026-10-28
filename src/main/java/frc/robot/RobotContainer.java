// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.indexer.hopper.HopperCommands;
import frc.robot.subsystems.indexer.hopper.HopperIO;
import frc.robot.subsystems.indexer.hopper.HopperIOReal;
import frc.robot.subsystems.indexer.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.kicker.KickerCommands;
import frc.robot.subsystems.indexer.kicker.KickerIO;
import frc.robot.subsystems.indexer.kicker.KickerIOReal;
import frc.robot.subsystems.indexer.kicker.KickerIOSim;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.intake.roller.RollerCommands;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOReal;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelCommands;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RobotContainer {

  private final HopperSubsystem hopper;
  private final KickerSubsystem kicker;
  private final RollerSubsystem roller;
  private final FlywheelSubsystem flywheel;

  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  public RobotContainer() {

    if (RobotBase.isReal()) {
      hopper = new HopperSubsystem(new HopperIOReal());
      kicker = new KickerSubsystem(new KickerIOReal());
      roller = new RollerSubsystem(new RollerIOReal());
      flywheel = new FlywheelSubsystem(new FlywheelIOReal());
    } else {
      if (RobotConstants.isReplay) {
        // Replay
        hopper = new HopperSubsystem(new HopperIO() {});
        kicker = new KickerSubsystem(new KickerIO() {});
        roller = new RollerSubsystem(new RollerIO() {});
        flywheel = new FlywheelSubsystem(new FlywheelIO() {});
      } else {
        hopper = new HopperSubsystem(new HopperIOSim());
        kicker = new KickerSubsystem(new KickerIOSim());
        roller = new RollerSubsystem(new RollerIOSim());
        flywheel = new FlywheelSubsystem(new FlywheelIOSim());
      }
    }
    configureBindings();
  }

  private void configureBindings() {
    driverController.circle().whileTrue(HopperCommands.runForwardPID(hopper));
    driverController.circle().whileTrue(KickerCommands.runForwardPID(kicker));
    driverController.circle().whileTrue(RollerCommands.runForwardPID(roller));
    driverController.circle().whileTrue(FlywheelCommands.runForwardPID(flywheel));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
