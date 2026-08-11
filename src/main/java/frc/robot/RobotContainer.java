// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.hopper.HopperIO;
import frc.robot.subsystems.indexer.hopper.HopperIOReal;
import frc.robot.subsystems.indexer.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.kicker.KickerIO;
import frc.robot.subsystems.indexer.kicker.KickerIOReal;
import frc.robot.subsystems.indexer.kicker.KickerIOSim;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;

public class RobotContainer {

  private final HopperSubsystem hopper;
  private final KickerSubsystem kicker;

  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      hopper = new HopperSubsystem(new HopperIOReal());
      kicker = new KickerSubsystem(new KickerIOReal());
    } else {
      if (RobotConstants.isReplay) {
        // Replay
        hopper = new HopperSubsystem(new HopperIO() {});
        kicker = new KickerSubsystem(new KickerIO() {});
      } else {
        hopper = new HopperSubsystem(new HopperIOSim());
        kicker = new KickerSubsystem(new KickerIOSim());
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    driverController.L1().onTrue(IndexerCommands.runShooting(hopper, kicker));
    driverController.R1().onTrue(IndexerCommands.stop(hopper, kicker));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
