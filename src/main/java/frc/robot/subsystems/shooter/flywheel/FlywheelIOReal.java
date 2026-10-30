package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FlywheelIOReal implements FlywheelIO {
  public final SparkMax flywheelSparkLeftLead =
      new SparkMax(FlywheelConstants.canIdLeftLead, MotorType.kBrushless);
  public final RelativeEncoder flywheelEncoderLeft = flywheelSparkLeftLead.getEncoder();
  public final SparkMax flywheelSparkLeftFollow =
      new SparkMax(FlywheelConstants.canIdLeftFollow, MotorType.kBrushless);

  public final SparkMax flywheelSparkRightLead =
      new SparkMax(FlywheelConstants.canIdRightLead, MotorType.kBrushless);
  public final RelativeEncoder flywheelEncoderRight = flywheelSparkRightLead.getEncoder();
  public final SparkMax flywheelSparkRightFollow =
      new SparkMax(FlywheelConstants.canIdRightFollow, MotorType.kBrushless);

  public FlywheelIOReal() {
    SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeadConfig = new SparkMaxConfig();

    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    // leftFollowConfig.follow(flywheelSparkLeftLead);
    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    // rightFollowConfig.follow(flywheelSparkRightLead);
    leftFollowConfig.inverted(true);
    rightFollowConfig.inverted(true);

    leftLeadConfig.smartCurrentLimit(40);
    rightLeadConfig.smartCurrentLimit(40);
    leftFollowConfig.smartCurrentLimit(40);
    rightFollowConfig.smartCurrentLimit(40);

    leftLeadConfig.idleMode(IdleMode.kCoast);
    rightLeadConfig.idleMode(IdleMode.kCoast);
    leftFollowConfig.idleMode(IdleMode.kCoast);
    rightFollowConfig.idleMode(IdleMode.kCoast);

    flywheelSparkLeftLead.configure(
        leftLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelSparkRightLead.configure(
        rightLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelSparkLeftFollow.configure(
        leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelSparkRightFollow.configure(
        rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.rotationsPerSecLeft =
        flywheelEncoderLeft.getVelocity() / FlywheelConstants.reduction / 60;
    inputs.appliedOutputLeft = flywheelSparkLeftLead.getAppliedOutput() * 12;

    inputs.rotationsPerSecRight =
        flywheelEncoderRight.getVelocity() / FlywheelConstants.reduction / 60;
    inputs.appliedOutputRight = flywheelSparkRightLead.getAppliedOutput() * 12;
  }

  @Override
  public void setVoltage(double voltageLeft, double voltageRight) {
    flywheelSparkLeftLead.setVoltage(voltageLeft * 1.08);
    flywheelSparkRightLead.setVoltage(voltageRight);

    flywheelSparkLeftFollow.setVoltage(voltageLeft * 1.08);
    flywheelSparkRightFollow.setVoltage(voltageRight);
  }

  @Override
  public void stop() {
    flywheelSparkLeftLead.setVoltage(0);
    flywheelSparkRightLead.setVoltage(0);

    flywheelSparkLeftFollow.setVoltage(0);
    flywheelSparkRightFollow.setVoltage(0);
  }
}
