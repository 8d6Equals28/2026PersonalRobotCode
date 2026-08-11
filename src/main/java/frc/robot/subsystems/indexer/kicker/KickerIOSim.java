package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.kicker.KickerIO.KickerIOInputs;

public class KickerIOSim implements KickerIO {
  private double simulatedOutput;
  private final DCMotorSim kickerMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  public KickerIOSim() {
    simulatedOutput = 0;
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    kickerMotorSim.update(0.02);
    inputs.rotationsPerSec =
        kickerMotorSim.getAngularVelocityRPM() / KickerConstants.REDUCTION / 60;
    inputs.appliedOutput = simulatedOutput;
  }

  @Override
  public void setSpeed(double speed) {
    speed *= KickerConstants.speed_multiplier;
    kickerMotorSim.setInputVoltage(speed);
    simulatedOutput = speed / 12;
  }
}
