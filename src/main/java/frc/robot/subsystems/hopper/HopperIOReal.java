package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOReal implements HopperIO{
    public final SparkMax hopperSpark = new SparkMax(HopperConstants.CAN_ID, MotorType.kBrushless);
    public final RelativeEncoder hopperEncoder = hopperSpark.getEncoder();

    public HopperIOReal() {
        hopperSpark.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.rotationsPerSec = hopperEncoder.getVelocity() / HopperConstants.REDUCTION / 60;
        inputs.appliedOutput = hopperSpark.getAppliedOutput();
    }

    @Override
    public void setVoltage(double voltage) {
        hopperSpark.setVoltage(voltage);
    }
}
