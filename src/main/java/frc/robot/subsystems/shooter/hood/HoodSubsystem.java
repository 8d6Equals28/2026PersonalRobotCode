package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.RollerConstants;

public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Constraints hoodProfile;
  private final ProfiledPIDController hoodPIDProfiled;
  private final SimpleMotorFeedforward hoodFF;

  public HoodSubsystem(HoodIO io) {
    if (RobotBase.isReal()) {
      hoodProfile = new TrapezoidProfile.Constraints(HoodConstants.maxVelReal, HoodConstants.maxAccReal);
      hoodPIDProfiled =
          new ProfiledPIDController(HoodConstants.kPReal, HoodConstants.kIReal, HoodConstants.kDReal, hoodProfile);
      hoodFF = new SimpleMotorFeedforward(
          HoodConstants.kSReal, HoodConstants.kVReal, HoodConstants.kAReal, 0.02);
    } else {
      hoodProfile = new TrapezoidProfile.Constraints(HoodConstants.maxVelSim, HoodConstants.maxAccSim);
      hoodPIDProfiled =
          new ProfiledPIDController(HoodConstants.kPSim, HoodConstants.kISim, HoodConstants.kDSim, hoodProfile);
      hoodFF = new SimpleMotorFeedforward(
          HoodConstants.kSSim, HoodConstants.kVSim, HoodConstants.kASim, 0.02);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setGoalPosition(double goalDegrees) {
    double prevVel = hoodPIDProfiled.getSetpoint().velocity;
    double outputPID = hoodPIDProfiled.calculate(inputs.position, goalDegrees);
    double nextVel = hoodPIDProfiled.getSetpoint().velocity;
    double outputFF = hoodFF.calculateWithVelocities(prevVel, nextVel);
    io.setVoltage(outputPID + outputFF);
  }

  public void stop() {
    io.stop();
  }
}