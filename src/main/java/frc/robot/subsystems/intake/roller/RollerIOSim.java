package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.roller.RollerIO.RollerIOInputs;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim rollerMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  public RollerIOSim() {}

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    rollerMotorSim.update(0.02);
    inputs.rotationsPerSec =
        rollerMotorSim.getAngularVelocityRPM() / RollerConstants.reduction / 60;
    inputs.appliedOutput = rollerMotorSim.getInputVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    rollerMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    rollerMotorSim.setInputVoltage(0);
  }
}
