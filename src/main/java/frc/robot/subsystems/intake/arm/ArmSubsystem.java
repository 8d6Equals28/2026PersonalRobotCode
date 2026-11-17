package frc.robot.subsystems.intake.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Constraints armProfile;
  private final ProfiledPIDController armPIDProfiled;
  private final SimpleMotorFeedforward armFF;

  public ArmSubsystem(ArmIO io) {
    if (RobotBase.isReal()) {
      armProfile =
          new TrapezoidProfile.Constraints(ArmConstants.maxVelReal, ArmConstants.maxAccReal);
      armPIDProfiled = new ProfiledPIDController(
          ArmConstants.kPReal, ArmConstants.kIReal, ArmConstants.kDReal, armProfile);
      armFF = new SimpleMotorFeedforward(
          ArmConstants.kSReal, ArmConstants.kVReal, ArmConstants.kAReal, 0.02);
    } else {
      armProfile =
          new TrapezoidProfile.Constraints(ArmConstants.maxVelSim, ArmConstants.maxAccSim);
      armPIDProfiled = new ProfiledPIDController(
          ArmConstants.kPSim, ArmConstants.kISim, ArmConstants.kDSim, armProfile);
      armFF = new SimpleMotorFeedforward(
          ArmConstants.kSSim, ArmConstants.kVSim, ArmConstants.kASim, 0.02);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setGoalPosition(double goalDegrees) {
    double prevVel = armPIDProfiled.getSetpoint().velocity;
    double outputPID = armPIDProfiled.calculate(inputs.position, goalDegrees);
    double nextVel = armPIDProfiled.getSetpoint().velocity;
    double outputFF = armFF.calculateWithVelocities(prevVel, nextVel);
    io.setVoltage(outputPID + outputFF);
  }

  public void stop() {
    io.stop();
  }
}
