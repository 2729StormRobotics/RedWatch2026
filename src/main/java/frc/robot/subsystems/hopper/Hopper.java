package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged(); 

  public Hopper(HopperIO io) {
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
   * Returns a command that runs the hopper at a constant voltage.
   */
  public Command runContinuous() {
    return this.run(() -> this.setVoltage(HopperConstants.KICK_VOLTAGE))
        .finallyDo(this::stop);
  }

  /**
   * Returns a command that runs the hopper in reverse (for clearing jams).
   */
  public Command reverse() {
    return this.run(() -> this.setVoltage(HopperConstants.REVERSE_VOLTAGE))
        .finallyDo(this::stop);
  }
    public Command stopCommand() {
    return this.run(() -> this.stop());
  }
}