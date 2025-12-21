package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Command to turn the robot to a specific angle using PID control.
 * The robot will rotate in place to reach the target angle.
 */
public class TurnAngleCommand extends Command {
  private final Drive drive;
  private final Rotation2d angle;

  /** PID controller for angle control */
  private static PIDController angleController =
      new PIDController(kTurnAngleP, kTurnAngleI, kTurnAngleD);

  /**
   * Creates a new TurnAngleCommand.
   *
   * @param drive The drive subsystem
   * @param angle The target angle to turn to
   */

  public TurnAngleCommand(Drive drive, Rotation2d angle) {
    addRequirements(drive);
    this.drive = drive;
    this.angle = angle;

    // Configure PID controller for continuous input (wraps around at -π to π)
    angleController.setTolerance(kTurnAngleTolerance, kTurnAngleRateTolerance);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    angleController.reset();
    angleController.setSetpoint(angle.getRadians());
  }

  @Override
  public void execute() {
    // No linear movement - only rotation
    double linearMagnitude = 0.0;
    Rotation2d linearDirection = new Rotation2d();

    // Calculate angular velocity using PID controller
    double omega = angleController.calculate(drive.getRotation().getRadians(), angle.getRadians());

    // Square values for better control feel
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calculate new linear velocity (will be zero for rotation in place)
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field-relative speeds & send command
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec(),
            drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}