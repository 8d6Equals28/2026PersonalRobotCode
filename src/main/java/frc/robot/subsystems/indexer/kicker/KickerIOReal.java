package frc.robot.subsystems.indexer.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIOReal implements KickerIO {
  public final SparkMax kickerSpark = new SparkMax(KickerConstants.CAN_ID, MotorType.kBrushless);
  public final RelativeEncoder kickerEncoder = kickerSpark.getEncoder();

  public KickerIOReal() {
    kickerSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.rotationsPerSec = kickerEncoder.getVelocity() / KickerConstants.REDUCTION / 60;
    inputs.appliedOutput = kickerSpark.getAppliedOutput();
  }

  @Override
  public void setSpeed(double speed) {
    speed *= KickerConstants.speed_multiplier;
    kickerSpark.setVoltage(speed);
  }
}
