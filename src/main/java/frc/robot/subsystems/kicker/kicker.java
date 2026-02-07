package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class kicker extends SubsystemBase {
  private final kickerIO io;
  private final kickerIOInputsAutoLogged inputs = new kickerIOInputsAutoLogged(); 

  public kicker(kickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  /**
   * Returns a command that runs the kicker at a constant voltage.
   */
  public Command runContinuous() {
    return this.run(() -> this.setVoltage(kickerConstants.KICK_VOLTAGE))
        .finallyDo(this::stop);
  }

  /**
   * Returns a command that runs the kicker in reverse (for clearing jams).
   */
  public Command reverse() {
    return this.run(() -> this.setVoltage(kickerConstants.REVERSE_VOLTAGE))
        .finallyDo(this::stop);
  }
}