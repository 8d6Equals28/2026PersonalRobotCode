package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;


import edu.wpi.first.math.system.plant.LinearSystemId;

public class HopperIOSim implements HopperIO {
    private double simulatedOutput;
    private final DCMotorSim hopperMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1),
        DCMotor.getNEO(1)
    );

    public HopperIOSim() {
        simulatedOutput = 0;
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        hopperMotorSim.update(0.02);
        inputs.rotationsPerSec = hopperMotorSim.getAngularVelocityRPM() / HopperConstants.REDUCTION / 60;
        inputs.appliedOutput = simulatedOutput;
    }

    @Override
    public void setVoltage(double voltage) {
        hopperMotorSim.setInputVoltage(voltage);
        simulatedOutput = voltage/12;
    }
}
