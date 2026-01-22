package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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
  }

  public void climb() {
    io.setLock(false);
    io.setVoltage(ClimbConstants.CLIMB_VOLTAGE);
  }

  public void retract() {
    io.setVoltage(-ClimbConstants.CLIMB_VOLTAGE);
  }

  public void lock() {
    io.setLock(true);
    io.stop();
  }

  public void stop() {
    io.stop();
  }

  public Command climbCommand() {
    return Commands.runOnce(() -> climb(), this);
  }
}
