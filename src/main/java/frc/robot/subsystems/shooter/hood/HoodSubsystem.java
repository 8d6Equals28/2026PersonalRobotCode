package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final Constraints hoodProfile;
  private final ProfiledPIDController hoodPIDProfiled;
  private final SimpleMotorFeedforward hoodFF;

  private final LoggedNetworkNumber kPTunable;
  private final LoggedNetworkNumber kITunable;
  private final LoggedNetworkNumber kDTunable;
  private final LoggedNetworkNumber kSTunable;
  private final LoggedNetworkNumber kVTunable;
  private final LoggedNetworkNumber kATunable;

  public HoodSubsystem(HoodIO io) {
    if (RobotBase.isReal()) {
      hoodProfile =
          new TrapezoidProfile.Constraints(HoodConstants.maxVelReal, HoodConstants.maxAccReal);
      hoodPIDProfiled = new ProfiledPIDController(
          HoodConstants.kPReal, HoodConstants.kIReal, HoodConstants.kDReal, hoodProfile);
      hoodFF = new SimpleMotorFeedforward(
          HoodConstants.kSReal, HoodConstants.kVReal, HoodConstants.kAReal, 0.02);
      kPTunable = new LoggedNetworkNumber("/Tuning/Hood/kP", HoodConstants.kPReal);
      kITunable = new LoggedNetworkNumber("/Tuning/Hood/kI", HoodConstants.kIReal);
      kDTunable = new LoggedNetworkNumber("/Tuning/Hood/kD", HoodConstants.kDReal);
      kSTunable = new LoggedNetworkNumber("/Tuning/Hood/kS", HoodConstants.kSReal);
      kVTunable = new LoggedNetworkNumber("/Tuning/Hood/kV", HoodConstants.kVReal);
      kATunable = new LoggedNetworkNumber("/Tuning/Hood/kA", HoodConstants.kAReal);
    } else {
      hoodProfile =
          new TrapezoidProfile.Constraints(HoodConstants.maxVelSim, HoodConstants.maxAccSim);
      hoodPIDProfiled = new ProfiledPIDController(
          HoodConstants.kPSim, HoodConstants.kISim, HoodConstants.kDSim, hoodProfile);
      hoodFF = new SimpleMotorFeedforward(
          HoodConstants.kSSim, HoodConstants.kVSim, HoodConstants.kASim, 0.02);
      kPTunable = new LoggedNetworkNumber("/Tuning/Hood/kP", HoodConstants.kPSim);
      kITunable = new LoggedNetworkNumber("/Tuning/Hood/kI", HoodConstants.kISim);
      kDTunable = new LoggedNetworkNumber("/Tuning/Hood/kD", HoodConstants.kDSim);
      kSTunable = new LoggedNetworkNumber("/Tuning/Hood/kS", HoodConstants.kSSim);
      kVTunable = new LoggedNetworkNumber("/Tuning/Hood/kV", HoodConstants.kVSim);
      kATunable = new LoggedNetworkNumber("/Tuning/Hood/kA", HoodConstants.kASim);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    if (kPTunable.get() != hoodPIDProfiled.getP()) {
      hoodPIDProfiled.setP(kPTunable.get());
    }
    if (kITunable.get() != hoodPIDProfiled.getI()) {
      hoodPIDProfiled.setI(kITunable.get());
    }
    if (kDTunable.get() != hoodPIDProfiled.getD()) {
      hoodPIDProfiled.setD(kDTunable.get());
    }
    if (kSTunable.get() != hoodFF.getKs()) {
      hoodFF.setKs(kSTunable.get());
    }
    if (kVTunable.get() != hoodFF.getKv()) {
      hoodFF.setKv(kVTunable.get());
    }
    if (kATunable.get() != hoodFF.getKa()) {
      hoodFF.setKa(kATunable.get());
    }

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
