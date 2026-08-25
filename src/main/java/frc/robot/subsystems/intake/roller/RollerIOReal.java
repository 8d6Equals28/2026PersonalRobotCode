package frc.robot.subsystems.intake.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerIOReal implements RollerIO {
  public final SparkMax hopperSpark = new SparkMax(RollerConstants.CAN_ID, MotorType.kBrushless);
  public final RelativeEncoder hopperEncoder = hopperSpark.getEncoder();

  public RollerIOReal() {
    hopperSpark.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rotationsPerSec = hopperEncoder.getVelocity() / RollerConstants.REDUCTION / 60;
    inputs.appliedOutput = hopperSpark.getAppliedOutput();
  }

  @Override
  public void setSpeed(double speed) {
    speed *= RollerConstants.speed_multiplier;
    hopperSpark.setVoltage(speed);
  }
}
