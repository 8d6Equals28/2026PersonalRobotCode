package frc.robot.subsystems.intake.arm;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmIOReal implements ArmIO {
  public final SparkMax armSpark = new SparkMax(ArmConstants.canId, MotorType.kBrushless);
  public final RelativeEncoder armEncoder = armSpark.getEncoder();

  public ArmIOReal() {
    armSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = armEncoder.getPosition() / ArmConstants.reduction * 360;
    inputs.appliedOutput = armSpark.getAppliedOutput() * 12;
  }

  @Override
  public void setVoltage(double voltage) {
    armSpark.setVoltage(voltage);
  }

  @Override
  public void stop() {
    armSpark.setVoltage(0);
  }
}
