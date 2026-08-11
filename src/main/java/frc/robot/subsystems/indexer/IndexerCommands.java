package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.hopper.HopperCommands;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.kicker.KickerCommands;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;

public class IndexerCommands {
    public static Command runShooting(HopperSubsystem hopper, KickerSubsystem kicker) {
        return(HopperCommands.runForwardFast(hopper)).alongWith(KickerCommands.runForwardFast(kicker));
    }
    public static Command stop(HopperSubsystem hopper, KickerSubsystem kicker) {
        return(HopperCommands.stop(hopper)).alongWith(KickerCommands.stop(kicker));
    }
}
