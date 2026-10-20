package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The kicker subsystem. This is the "brain" of the mechanism: it decides WHAT the motor should do
 * and asks the IO layer to do it. It never touches motor controllers directly.
 *
 * <p>Flow of the whole robot, for reference:
 *
 * <pre>
 *   Controller button -> Command (in this class) -> Subsystem method -> IO interface
 *                                                                          |
 *                                      kickerIOReal (real robot) or kickerIOSim (simulator)
 * </pre>
 */
public class kicker extends SubsystemBase {
  // The IO object is how we talk to the hardware. Whether it is real or simulated is decided in
  // RobotContainer, so this class does not need to know or care.
  private final kickerIO io;

  // Holds the latest sensor readings. kickerIOInputsAutoLogged is GENERATED for us at build time
  // from the @AutoLog class inside kickerIO, so it will not exist until the project compiles once.
  private final kickerIOInputsAutoLogged inputs = new kickerIOInputsAutoLogged();

  public kicker(kickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // TODO: periodic() runs every 20 ms, automatically, as long as the subsystem exists.
    //   1. Ask the IO to refresh our inputs:      io.updateInputs(inputs);
    //   2. Send those inputs to AdvantageKit:     Logger.processInputs("Kicker", inputs);
    //      (needs: import org.littletonrobotics.junction.Logger;)
    //   Always do these two in this order, and do them first, so everything else in the
    //   subsystem works from fresh data.
  }

  // TODO: setPercent(double percent)
  //   A plain method that forwards to the IO:  io.setPercentIO(percent);
  //   Percent is -1.0 (full reverse) to 1.0 (full forward).

  // TODO: stop()
  //   A plain method that forwards to the IO:  io.stop();

  // TODO: Command runContinuous()
  //   Returns a command that kicks the ball while it is scheduled and stops when it ends.
  //   Hint: use this.run(() -> ...) to repeat an action every loop, then chain
  //   .finallyDo(...) so the motor is guaranteed to stop when the command ends or is interrupted.
  //   Use kickerConstants.KICK_VOLTAGE for the speed.
  //   (needs: import edu.wpi.first.wpilibj2.command.Command;)

  // TODO: Command reverse()
  //   Same shape as runContinuous(), but spins the other way, to clear jams.
  //   Use kickerConstants.REVERSE_VOLTAGE for the speed.
}
