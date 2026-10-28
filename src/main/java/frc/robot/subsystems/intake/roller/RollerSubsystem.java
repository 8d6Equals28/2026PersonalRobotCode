package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private final PIDController rollerPID;
  private final SimpleMotorFeedforward rollerFF;

  public RollerSubsystem(RollerIO io) {

    if (RobotBase.isReal()) {
      rollerPID =
          new PIDController(RollerConstants.kPReal, RollerConstants.kIReal, RollerConstants.kDReal);
      rollerFF = new SimpleMotorFeedforward(
          RollerConstants.kSReal, RollerConstants.kVReal, RollerConstants.kAReal, 0.02);
    } else {
      rollerPID =
          new PIDController(RollerConstants.kPSim, RollerConstants.kISim, RollerConstants.kDSim);
      rollerFF = new SimpleMotorFeedforward(
          RollerConstants.kSSim, RollerConstants.kVSim, RollerConstants.kASim, 0.02);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public void setGoalVel(double goalVel) {
    double voltage =
        rollerPID.calculate(inputs.rotationsPerSec, goalVel) + rollerFF.calculate(goalVel);
    voltage = MathUtil.clamp(voltage, -8.0, 8.0);
    io.setVoltage(voltage);
  }

  public void stop() {
    io.stop();
  }
}
