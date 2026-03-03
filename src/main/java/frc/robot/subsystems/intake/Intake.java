package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double desiredPivotPosition = IntakeConstants.RETRACTED_POSITION;
  private double desiredRollerPercent = 0.0;
  private boolean autoRetractEnabled = true;

  private TrapezoidProfile.State m_lastState = new TrapezoidProfile.State(0.0, 0.0);

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          IntakeConstants.kMaxVelocity,
          IntakeConstants.kMaxAcceleration));

  // 2D Visualization
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3.0, 3.0);
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d pivotArm;

  public Intake(IntakeIO io) {
    this.io = io;

    root = mechanism.getRoot("IntakeRoot", 1.5, 2.5);
    pivotArm = root.append(
        new LoggedMechanismLigament2d(
            "PivotArm",
            0.8,
            90.0,
            6.0,
            new Color8Bit(Color.kBlue)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (autoRetractEnabled && inputs.beamBreakTriggered) {
      retract();
    }

    TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredPivotPosition, 0.0);
    m_lastState = m_profile.calculate(0.020, m_lastState, goal);
    SmartDashboard.putNumber("intake/desiredpivotpos", m_lastState.position);
    io.setPivotPosition(desiredPivotPosition);
    io.setRollerPercent(desiredRollerPercent);

    // Update 2D Mechanism
    double angleDegrees = 90.0 - (inputs.pivotPositionRotations * 360.0);
    pivotArm.setAngle(angleDegrees);
    Logger.recordOutput("Intake/angle", angleDegrees);

    // --- 3D VISUALIZATION INTEGRATION ---
    // Pass the rotation to the master visualizer.
    // This will also trigger the hopper extension logic.
    AlphaMechanism3d.getInstance().setIntake(Rotation2d.fromRotations(inputs.pivotPositionRotations));

    // In a multi-subsystem bot, only one subsystem needs to call log()
    // to push the final Pose3d array to AdvantageScope.
    AlphaMechanism3d.getInstance().log();
  }

  public void deploy() {
    desiredPivotPosition = IntakeConstants.DEPLOYED_POSITION;
  }

  public void retract() {
    desiredPivotPosition = IntakeConstants.RETRACTED_POSITION;
    desiredRollerPercent = 0.0;
  }

  public void intake() {
    desiredRollerPercent = IntakeConstants.INTAKE_ROLLER_SPEED;
  }

  public void outake() {
    desiredRollerPercent = IntakeConstants.OUTAKE_ROLLER_SPEED;
  }

  public void intake(double pwr) {
    desiredRollerPercent = pwr;
  }

  public void eject() {
    desiredRollerPercent = IntakeConstants.EJECT_ROLLER_SPEED;
  }

  public void stopRoller() {
    desiredRollerPercent = 0.0;
  }

  @AutoLogOutput(key = "Intake/IsDeployed")
  public boolean isDeployed() {
    return Math
        .abs(inputs.pivotPositionRotations - IntakeConstants.DEPLOYED_POSITION) < IntakeConstants.POSITION_TOLERANCE;
  }

  public void stop() {
    io.stop();
  }

  public boolean decrementBall() {
    return io.decrementBall();
  }

  public Command deployCommand() {
    return Commands.runOnce(this::deploy, this);
  }

  public Command retractCommand() {
    return Commands.runOnce(this::retract, this);
  }

  public Command intakeCommand() {
    return Commands.run(() -> {
      intake();
    }, this);
  }

  public Command OutakeCommand() {
    return Commands.run(() -> {
      eject();
    }, this);
  }

  public Command intakeCommandTrigger(DoubleSupplier pwr) {
    return Commands.run(() -> {
      intake(pwr.getAsDouble());
    }, this);
  }

  public Command stopCommand() {
    return Commands.run(() -> {
      stop();
    }, this);
  }

}