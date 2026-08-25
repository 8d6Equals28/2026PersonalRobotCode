package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  //private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    //io.updateInputs(inputs);
    //Logger.processInputs("Roller", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
}
