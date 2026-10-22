package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final PIDController hopperPID;
  private final SimpleMotorFeedforward hopperFF;

  public HopperSubsystem(HopperIO io) {

    if (RobotBase.isReal()) {
      hopperPID =
          new PIDController(HopperConstants.kPReal, HopperConstants.kIReal, HopperConstants.kDReal);
      hopperFF = new SimpleMotorFeedforward(
          HopperConstants.kSReal, HopperConstants.kVReal, HopperConstants.kAReal, 0.02);
    } else {
      hopperPID =
          new PIDController(HopperConstants.kPSim, HopperConstants.kISim, HopperConstants.kDSim);
      hopperFF = new SimpleMotorFeedforward(
          HopperConstants.kSSim, HopperConstants.kVSim, HopperConstants.kASim, 0.02);
    }

    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setGoalVel(double goalVel) {
    double voltage =
        hopperPID.calculate(inputs.rotationsPerSec, goalVel) + hopperFF.calculate(goalVel);
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    io.setVoltage(voltage);
  }
}
