package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim flywheelMotorSimLeft = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  private final DCMotorSim flywheelMotorSimRight = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelMotorSimLeft.update(0.02);

    flywheelMotorSimRight.update(0.02);

    inputs.rotationsPerSecLeft =
        flywheelMotorSimLeft.getAngularVelocityRPM() / FlywheelConstants.reduction / 60;

    inputs.rotationsPerSecRight = flywheelMotorSimRight.getAngularVelocityRPM() / FlywheelConstants.reduction / 60;
    
    inputs.appliedOutputLeft = flywheelMotorSimLeft.getInputVoltage();
    inputs.appliedOutputRight = flywheelMotorSimRight.getInputVoltage();
  }

  @Override
  public void setVoltage(double voltageLeft, double voltageRight) {
    flywheelMotorSimLeft.setInputVoltage(voltageLeft);
    flywheelMotorSimRight.setInputVoltage(voltageRight);
  }

  @Override
  public void stop() {
    flywheelMotorSimLeft.setInputVoltage(0);
    flywheelMotorSimRight.setInputVoltage(0);
  }
}
