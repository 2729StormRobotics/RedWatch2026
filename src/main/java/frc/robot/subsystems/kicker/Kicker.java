package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.shooter.Shooter;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final Shooter shooter;

  public Kicker(KickerIO io, Shooter shooter) {
    this.io = io;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public void fire() {
    if (shooter.isReadyToFire()) {
      io.setVoltage(KickerConstants.FIRE_VOLTAGE);
    }
  }

  public void reverse() {
    io.setVoltage(-KickerConstants.FIRE_VOLTAGE);
  }

  public void stop() {
    io.stop();
  }

  public Command fireCommand() {
    return Commands.runOnce(() -> fire(), this);
  }
}
