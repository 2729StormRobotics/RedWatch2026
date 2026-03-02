package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

/**
 * Command group: aim the shooter toward our depot for passes to our side.
 * While running, the shooter uses pose-based aiming but targets the alliance-flipped depot
 * center instead of the hub. When the command ends, depot-aim mode is disabled and the
 * shooter is stopped.
 */
public final class HopperBackwardsIntake {

  private HopperBackwardsIntake() {}

  /**
   * Builds a command that enables depot-aim mode and keeps it active until interrupted.
   * On end, depot-aim is disabled and the shooter is stopped.
   *
   * @param shooter Shooter subsystem
   * @return command that aims at our depot until cancelled
   */
  public static Command getCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(hopper.runContinuous(), intake.OutakeCommand()).withName("backwards");
  }
}

