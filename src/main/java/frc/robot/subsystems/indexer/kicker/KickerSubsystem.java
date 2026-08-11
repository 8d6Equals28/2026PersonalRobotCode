package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.indexer.kicker.KickerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }
}
