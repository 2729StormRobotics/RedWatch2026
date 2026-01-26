package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();


  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    AlphaMechanism3d.getInstance().setClimb(inputs.positionMeters);
  }

  /**
   * Returns a command that runs the motor at the specified voltage while active,
   * and stops the motor immediately when the command ends/is released.
   */
  public Command runClimbVoltage(double volts) {
    return this.startEnd(
        () -> io.setVoltage(volts),
        () -> io.stop()
    );
  }

  public Command climbCommand() {
    return runClimbVoltage(ClimbConstants.CLIMB_VOLTAGE);
  }

  public Command retractCommand() {
    return runClimbVoltage(-ClimbConstants.CLIMB_VOLTAGE);
  }
}