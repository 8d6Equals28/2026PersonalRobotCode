package frc.robot.subsystems.hopper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputs inputs = new HopperIOInputs();

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //Logger.processInputs("Hopper", inputs);
    }


    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }
    
}
