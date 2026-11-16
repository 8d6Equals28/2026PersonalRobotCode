package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      HoodConstants.reduction,
      0.013,
      0.05,
      0,
      Units.degreesToRadians(HoodConstants.maxHeight),
      false,
      0);
  private double inputVoltage = 0;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodSim.update(0.02);
    inputs.position = hoodSim.getAngleRads() / 3.141592653589 * 180;
    inputs.appliedOutput = inputVoltage;
  }

  @Override
  public void setVoltage(double voltage) {
    inputVoltage = voltage;
    hoodSim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    inputVoltage = 0;
    hoodSim.setInputVoltage(0);
  }
}
