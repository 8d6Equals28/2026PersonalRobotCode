package frc.robot.subsystems.intake.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Constraints armProfile;
  private final ProfiledPIDController armPIDProfiled;
  private final SimpleMotorFeedforward armFF;

  private final LoggedNetworkNumber kPTunable;
  private final LoggedNetworkNumber kITunable;
  private final LoggedNetworkNumber kDTunable;
  private final LoggedNetworkNumber kSTunable;
  private final LoggedNetworkNumber kVTunable;
  private final LoggedNetworkNumber kATunable;

  public ArmSubsystem(ArmIO io) {
    if (RobotBase.isReal()) {
      armProfile =
          new TrapezoidProfile.Constraints(ArmConstants.maxVelReal, ArmConstants.maxAccReal);
      armPIDProfiled = new ProfiledPIDController(
          ArmConstants.kPReal, ArmConstants.kIReal, ArmConstants.kDReal, armProfile);
      armFF = new SimpleMotorFeedforward(
          ArmConstants.kSReal, ArmConstants.kVReal, ArmConstants.kAReal, 0.02);
      kPTunable = new LoggedNetworkNumber("/Tuning/Arm/kP", ArmConstants.kPReal);
      kITunable = new LoggedNetworkNumber("/Tuning/Arm/kI", ArmConstants.kIReal);
      kDTunable = new LoggedNetworkNumber("/Tuning/Arm/kD", ArmConstants.kDReal);
      kSTunable = new LoggedNetworkNumber("/Tuning/Arm/kS", ArmConstants.kSReal);
      kVTunable = new LoggedNetworkNumber("/Tuning/Arm/kV", ArmConstants.kVReal);
      kATunable = new LoggedNetworkNumber("/Tuning/Arm/kA", ArmConstants.kAReal);
    } else {
      armProfile =
          new TrapezoidProfile.Constraints(ArmConstants.maxVelSim, ArmConstants.maxAccSim);
      armPIDProfiled = new ProfiledPIDController(
          ArmConstants.kPSim, ArmConstants.kISim, ArmConstants.kDSim, armProfile);
      armFF = new SimpleMotorFeedforward(
          ArmConstants.kSSim, ArmConstants.kVSim, ArmConstants.kASim, 0.02);
      kPTunable = new LoggedNetworkNumber("/Tuning/Arm/kP", ArmConstants.kPSim);
      kITunable = new LoggedNetworkNumber("/Tuning/Arm/kI", ArmConstants.kISim);
      kDTunable = new LoggedNetworkNumber("/Tuning/Arm/kD", ArmConstants.kDSim);
      kSTunable = new LoggedNetworkNumber("/Tuning/Arm/kS", ArmConstants.kSSim);
      kVTunable = new LoggedNetworkNumber("/Tuning/Arm/kV", ArmConstants.kVSim);
      kATunable = new LoggedNetworkNumber("/Tuning/Arm/kA", ArmConstants.kASim);
    }
    this.io = io;
  }

  @Override
  public void periodic() {
    if (kPTunable.get() != armPIDProfiled.getP()) {
      armPIDProfiled.setP(kPTunable.get());
    }
    if (kITunable.get() != armPIDProfiled.getI()) {
      armPIDProfiled.setI(kITunable.get());
    }
    if (kDTunable.get() != armPIDProfiled.getD()) {
      armPIDProfiled.setD(kDTunable.get());
    }
    if (kSTunable.get() != armFF.getKs()) {
      armFF.setKs(kSTunable.get());
    }
    if (kVTunable.get() != armFF.getKv()) {
      armFF.setKv(kVTunable.get());
    }
    if (kATunable.get() != armFF.getKa()) {
      armFF.setKa(kATunable.get());
    }

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
