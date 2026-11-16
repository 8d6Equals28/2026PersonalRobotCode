package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HoodIOReal implements HoodIO {
  public final SparkMax hoodSpark = new SparkMax(HoodConstants.canId, MotorType.kBrushless);
  public final RelativeEncoder hoodEncoder = hoodSpark.getEncoder();

  public HoodIOReal() {
    hoodSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = hoodEncoder.getPosition() / HoodConstants.reduction * 360;
    inputs.appliedOutput = hoodSpark.getAppliedOutput() * 12;
  }

  @Override
  public void setVoltage(double voltage) {
    hoodSpark.setVoltage(voltage);
  }

  @Override
  public void stop() {
    hoodSpark.setVoltage(0);
  }
}
