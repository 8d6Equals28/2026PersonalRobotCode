// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.indexer.hopper.HopperCommands;
import frc.robot.subsystems.indexer.hopper.HopperIOReal;
import frc.robot.subsystems.indexer.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;

public class RobotContainer {

  private final HopperSubsystem hopper;

  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  public RobotContainer() {
    if (RobotBase.isReal()) {
      hopper = new HopperSubsystem(new HopperIOReal());
    }
    else {
      if (RobotConstants.isReplay) {
        //Replay
        hopper = null;
      }
      else {
        hopper = new HopperSubsystem(new HopperIOSim());
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    driverController.L2().whileTrue(HopperCommands.runForwardFast(hopper));
    driverController.L1().whileTrue(HopperCommands.runForwardSlow(hopper));
    driverController.R2().whileTrue(HopperCommands.runBackward(hopper));
    driverController.R1().whileTrue(HopperCommands.stop(hopper));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
