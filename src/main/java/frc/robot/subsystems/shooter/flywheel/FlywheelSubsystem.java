package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final PIDController flywheelPIDLeft;
  private final SimpleMotorFeedforward flywheelFFLeft;

  private final PIDController flywheelPIDRight;
  private final SimpleMotorFeedforward flywheelFFRight;

  public FlywheelSubsystem(FlywheelIO io) {

    if (RobotBase.isReal()) {
      flywheelPIDLeft = new PIDController(
          FlywheelConstants.kPReal, FlywheelConstants.kIReal, FlywheelConstants.kDReal);
      flywheelFFLeft = new SimpleMotorFeedforward(
          FlywheelConstants.kSReal, FlywheelConstants.kVReal, FlywheelConstants.kAReal, 0.02);

      flywheelPIDRight = new PIDController(
          FlywheelConstants.kPReal, FlywheelConstants.kIReal, FlywheelConstants.kDReal);
      flywheelFFRight = new SimpleMotorFeedforward(
          FlywheelConstants.kSReal, FlywheelConstants.kVReal, FlywheelConstants.kAReal, 0.02);
    } else {
      flywheelPIDLeft = new PIDController(
          FlywheelConstants.kPSim, FlywheelConstants.kISim, FlywheelConstants.kDSim);
      flywheelFFLeft = new SimpleMotorFeedforward(
          FlywheelConstants.kSSim, FlywheelConstants.kVSim, FlywheelConstants.kASim, 0.02);

      flywheelPIDRight = new PIDController(
          FlywheelConstants.kPSim, FlywheelConstants.kISim, FlywheelConstants.kDSim);
      flywheelFFRight = new SimpleMotorFeedforward(
          FlywheelConstants.kSSim, FlywheelConstants.kVSim, FlywheelConstants.kASim, 0.02);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void setGoalVel(double goalVel) {
    double voltageLeft = flywheelPIDLeft.calculate(inputs.rotationsPerSecLeft, goalVel)
        + flywheelFFLeft.calculate(goalVel);
    voltageLeft = MathUtil.clamp(voltageLeft, -8.0, 8.0);

    double voltageRight = flywheelPIDRight.calculate(inputs.rotationsPerSecRight, goalVel)
        + flywheelFFRight.calculate(goalVel);
    voltageRight = MathUtil.clamp(voltageRight, -8.0, 8.0);

    io.setVoltage(voltageLeft, voltageRight);
  }

  public void stop() {
    io.stop();
  }
}
