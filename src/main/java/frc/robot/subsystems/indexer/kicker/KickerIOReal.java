package frc.robot.subsystems.indexer.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIOReal implements KickerIO {
  public final SparkMax kickerSpark = new SparkMax(KickerConstants.canId, MotorType.kBrushless);
  public final RelativeEncoder kickerEncoder = kickerSpark.getEncoder();

  public KickerIOReal() {
    kickerSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.rotationsPerSec = kickerEncoder.getVelocity() / KickerConstants.reduction / 60;
    inputs.appliedOutput = kickerSpark.getAppliedOutput() * 12;
  }

  @Override
  public void setVoltage(double voltage) {
    kickerSpark.setVoltage(voltage);
  }

  @Override
  public void stop() {
    kickerSpark.setVoltage(0);
  }
}
