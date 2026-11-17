package frc.robot.subsystems.intake.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      ArmConstants.reduction,
      0.013,
      0.05,
      0,
      Double.POSITIVE_INFINITY,
      false,
      0);
  private double inputVoltage = 0;

  public ArmIOSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);
    inputs.position = armSim.getAngleRads() / 3.141592653589 * 180;
    inputs.appliedOutput = inputVoltage;
  }

  @Override
  public void setVoltage(double voltage) {
    inputVoltage = voltage;
    armSim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    inputVoltage = 0;
    armSim.setInputVoltage(0);
  }
}
