package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.hopper.HopperIO.HopperIOInputs;

public class HopperIOSim implements HopperIO {
  private double simulatedOutput;
  private final DCMotorSim hopperMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  public HopperIOSim() {
    simulatedOutput = 0;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperMotorSim.update(0.02);
    inputs.rotationsPerSec =
        hopperMotorSim.getAngularVelocityRPM() / HopperConstants.REDUCTION / 60;
    inputs.appliedOutput = simulatedOutput;
  }

  @Override
  public void setSpeed(double speed) {
    speed *= HopperConstants.speed_multiplier;
    hopperMotorSim.setInputVoltage(speed);
    simulatedOutput = speed / 12;
  }
}
