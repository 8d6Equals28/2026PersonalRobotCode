package frc.robot.subsystems.indexer.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.roller.RollerIO.RollerIOInputs;

public class RollerIOSim implements RollerIO {
  private double simulatedOutput;
  private final DCMotorSim rollerMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  public RollerIOSim() {
    simulatedOutput = 0;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    rollerMotorSim.update(0.02);
    inputs.rotationsPerSec =
        rollerMotorSim.getAngularVelocityRPM() / RollerConstants.REDUCTION / 60;
    inputs.appliedOutput = simulatedOutput;
  }

  @Override
  public void setSpeed(double speed) {
    speed *= RollerConstants.speed_multiplier;
    rollerMotorSim.setInputVoltage(speed);
    simulatedOutput = speed / 12;
  }
}
