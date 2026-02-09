package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.kicker.kicker;

/**
 * Command group: run hopper and kicker together to feed and shoot.
 */
public final class ShootingCommand {

  private ShootingCommand() {}

  /**
   * Builds a command that runs the hopper and kicker in parallel (feed + shoot).
   *
   * @param hopper Hopper subsystem
   * @param kicker Kicker subsystem
   * @return command that runs both until interrupted
   */
  public static Command getCommand(Hopper hopper, kicker kicker) {
    return Commands.parallel(
            hopper.runContinuous(),
            kicker.runContinuous())
        .withName("Shooting");
  }
}
