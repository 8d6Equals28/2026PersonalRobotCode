package frc.robot.subsystems.intake.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerIOReal implements RollerIO {
  public final SparkMax rollerSpark = new SparkMax(RollerConstants.canId, MotorType.kBrushless);
  public final RelativeEncoder rollerEncoder = rollerSpark.getEncoder();

  public RollerIOReal() {
    rollerSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rotationsPerSec = rollerEncoder.getVelocity() / RollerConstants.reduction / 60;
    inputs.appliedOutput = rollerSpark.getAppliedOutput() * 12;
  }

  @Override
  public void setVoltage(double voltage) {
    rollerSpark.setVoltage(voltage);
  }

  @Override
  public void stop() {
    rollerSpark.setVoltage(0);
  }
}
