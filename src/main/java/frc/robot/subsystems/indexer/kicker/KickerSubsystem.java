package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final PIDController kickerPID;
  private final SimpleMotorFeedforward kickerFF;

  public KickerSubsystem(KickerIO io) {
    if (RobotBase.isReal()) {
      kickerPID =
          new PIDController(KickerConstants.kPReal, KickerConstants.kIReal, KickerConstants.kDReal);
      kickerFF = new SimpleMotorFeedforward(
          KickerConstants.kSReal, KickerConstants.kVReal, KickerConstants.kAReal, 0.02);
    } else {
      kickerPID =
          new PIDController(KickerConstants.kPSim, KickerConstants.kISim, KickerConstants.kDSim);
      kickerFF = new SimpleMotorFeedforward(
          KickerConstants.kSSim, KickerConstants.kVSim, KickerConstants.kASim, 0.02);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public void setGoalVel(double goalVel) {
    double voltage =
        kickerPID.calculate(inputs.rotationsPerSec, goalVel) + kickerFF.calculate(goalVel);
    voltage = MathUtil.clamp(voltage, -8.0, 8.0);
    io.setVoltage(voltage);
  }

  public void stop() {
    io.stop();
  }
}
