package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Command group: constantly aim the shooter (hood + turret from move-and-shoot, flywheel).
 * While running, the shooter's periodic() updates turret angle, hood angle, and flywheel
 * RPM based on robot pose. When the command ends, aiming is disabled and the shooter stops.
 */
public final class AimCommand {

  private AimCommand() {}

  /**
   * Builds a command that enables move-and-shoot (continuous hood + turret aim) and flywheel.
   * Runs until interrupted. On end, disables move-and-shoot and stops the shooter.
   *
   * @param shooter Shooter subsystem
   * @return command that aims until cancelled
   */
  public static Command getCommand(Shooter shooter) {
    return Commands.sequence(
            Commands.runOnce(shooter::enableMoveAndShoot, shooter),
            Commands.run(
                    () -> {},
                    shooter)
                .finallyDo(
                    () -> {
                      shooter.disableMoveAndShoot();
                      shooter.stop();
                    }))
        .withName("Aim");
  }
}
