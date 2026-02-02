// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.lang.reflect.Field;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretIOReal;

/**
 * Shooter super-subsystem that coordinates Flywheel, Hood, and Turret.
 * Implements Move-and-Shoot logic with dynamic heading/RPM/Angle calculation
 * based on robot velocity vectors.
 */
public class Shooter extends SubsystemBase {
  // IO interfaces
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final TurretIO turretIO;

  // Drive subsystem reference for robot pose
  private final Drive drive;

  // Inputs
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  // State variables
  private double desiredFlywheelVelocity = 0.0;
  private double desiredHoodAngle = 0.0;
  private double desiredTurretAngle = 0.0;

  private boolean prep = true;

  // Move-and-Shoot state
  private boolean moveAndShootEnabled = false;

  // Sim?
  private AbstractDriveTrainSimulation driveTrainSimulation;

  // 3D Mechanism visualization
  private final AlphaMechanism3d mechanism3d = AlphaMechanism3d.getInstance();

  /**
   * Creates a new Shooter super-subsystem.
   *
   * @param flywheelIO Flywheel IO implementation
   * @param hoodIO     Hood IO implementation
   * @param turretIO   Turret IO implementation
   * @param drive      Drive subsystem for robot pose
   */
  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO, Drive drive,
      AbstractDriveTrainSimulation driveTrainSimulation) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;
    this.drive = drive;
    this.driveTrainSimulation = driveTrainSimulation;

    this.setDefaultCommand(idleFlywheelCommand());
  }

@Override
  public void periodic() {
    // Update inputs from hardware
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);
    turretIO.updateInputs(turretInputs);

    // Process inputs for logging
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
    Logger.processInputs("Shooter/Turret", turretInputs);

    if (moveAndShootEnabled) {
      updateMoveAndShoot(prep);
    }
    // 3. Trench Safety Override
    // If we are under the trench, we OVERWRITE the hood setpoint calculated by
    // updateMoveAndShoot
    boolean safetyActive = isUnderTrench();
    if (safetyActive) {
      // Force the hood down to clear the 22.25" opening height
      desiredHoodAngle = frc.robot.subsystems.shooter.hood.HoodConstants.MIN_ANGLE_RAD;
    }

    Logger.recordOutput("Shooter/TrenchSafetyActive", safetyActive);

    // Apply desired setpoints
    if (desiredFlywheelVelocity != 0.0) {
      flywheelIO.setVelocity(desiredFlywheelVelocity);
    }

    // Always update hood setpoint
    hoodIO.setAngle(desiredHoodAngle);

    // Always update turret setpoint
    turretIO.setAngle(desiredTurretAngle);

    // Log shooter state
    Logger.recordOutput("Shooter/ReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/MoveAndShootEnabled", moveAndShootEnabled);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", desiredHoodAngle);
    Logger.recordOutput("Shooter/Turret/SetpointAngle", desiredTurretAngle);

    // Update 3D mechanism visualization
    // Turret angle is robot-relative (0° = robot forward, positive = CCW)
    mechanism3d.setShooter(new Rotation2d(getTurretCurrentAngle()), new Rotation2d(getHoodCurrentAngle()));
    // Log the mechanism poses
    mechanism3d.log();
  }

  /**
   * Updates Move-and-Shoot calculations based on robot pose and velocity.
   * Calculates heading to hub and adjusts turret/hood/flywheel accordingly.
   */
  private void updateMoveAndShoot(boolean isPrep) {
    // --- START MODIFICATION: SIM-AWARE POSE RETRIEVAL ---
    Pose2d robotPose;
    ChassisSpeeds robotVelocity;

    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM && driveTrainSimulation != null) {
      // Use Ground Truth from MapleSim for perfect simulation tracking
      robotPose = driveTrainSimulation.getSimulatedDriveTrainPose();
      robotVelocity = driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    } else {
      // Use Estimated Pose (Odometry/Vision) for Real or Replay
      robotPose = drive.getPose();
      robotVelocity = drive.getChassisSpeeds();
    }
    // --- END MODIFICATION ---

    // Get hub center position (alliance-relative)
    Translation2d hubCenter;

    //seems to always be false
    boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
    // Note: In 2026 REBUILT, ensure FieldConstants.Hub reflects the target you want
    // to hit
    if (isFlipped) {
      hubCenter = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    } else {
      hubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    }

    // Calculate vector from robot to hub (in field coordinates)
    Translation2d robotToHub = hubCenter.minus(robotPose.getTranslation());

    // Check if target is valid (non-zero distance)
    double distanceToHub = robotToHub.getNorm();
    if (distanceToHub < 0.01) {
      disableMoveAndShoot();
      return;
    }

    // Calculate heading to hub in field coordinates
    Rotation2d headingToHub = robotToHub.getAngle();

    // Calculate required turret angle relative to robot forward direction
    Rotation2d robotRotation = robotPose.getRotation();
    Rotation2d turretRotation = headingToHub.minus(robotRotation);

    // Normalize and clamp
    double turretAngle = MathUtil.inputModulus(turretRotation.getRadians(), -Math.PI, Math.PI);
    turretAngle = MathUtil.clamp(turretAngle, TurretConstants.MIN_ANGLE_RAD, TurretConstants.MAX_ANGLE_RAD);
    setTurretAngle(turretAngle);

    // Calculate required flywheel RPM (now using field-relative velocity for better
    // compensation)
    double requiredRPM = calculateRequiredRPM(distanceToHub, robotVelocity);
    setFlywheelVelocity(requiredRPM / 60.0);
    setHoodAngleFromDistance(distanceToHub);

    // Log target information
    Logger.recordOutput("Shooter/isPrep", isPrep);
    Logger.recordOutput("Shooter/TargetDistance", distanceToHub);
    Logger.recordOutput("Shooter/TargetHeading", headingToHub.getDegrees());
    Logger.recordOutput("Shooter/TurretSetpoint", Math.toDegrees(turretAngle));
    Logger.recordOutput("Shooter/SimPoseUsed", frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM);
  }

  private boolean isUnderTrench() {
    Pose2d currentPose;
    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM && driveTrainSimulation != null) {
      currentPose = driveTrainSimulation.getSimulatedDriveTrainPose();
    } else {
      currentPose = drive.getPose();
    }

    double x = currentPose.getX();
    double y = currentPose.getY();

    // Calculate Bounds
    double xMin = FieldConstants.LinesVertical.hubCenter - (FieldConstants.LeftTrench.depth / 2.0);
    double xMax = FieldConstants.LinesVertical.hubCenter + (FieldConstants.LeftTrench.depth / 2.0);
    double margin = 0.35;

    // Define Y boundaries based on Trench width
    double leftTrenchInnerY = FieldConstants.fieldWidth - FieldConstants.LeftTrench.width;
    double rightTrenchInnerY = FieldConstants.RightTrench.width;

    // Log the bounds for debugging
    Logger.recordOutput("Shooter/TrenchBounds/XRange", new double[] { xMin - margin, xMax + margin });
    Logger.recordOutput("Shooter/TrenchBounds/LeftYLimit", leftTrenchInnerY);
    Logger.recordOutput("Shooter/TrenchBounds/RightYLimit", rightTrenchInnerY);

    boolean withinX = (x >= xMin - margin) && (x <= xMax + margin);
    boolean inLeftTrenchY = (y >= leftTrenchInnerY);
    boolean inRightTrenchY = (y <= rightTrenchInnerY);

    return withinX && (inLeftTrenchY || inRightTrenchY);
  }

  /**
   * Calculates required flywheel RPM based on distance and robot velocity.
   */
  private double calculateRequiredRPM(double distanceMeters, ChassisSpeeds robotVelocity) {
    // Simplified calculation - actual implementation should use proper ballistics
    double baseRPM = 3000.0 + (distanceMeters * 200.0); // Simplified linear relationship

    // Account for robot velocity
    double velocityComponent = Math.sqrt(
        robotVelocity.vxMetersPerSecond * robotVelocity.vxMetersPerSecond +
            robotVelocity.vyMetersPerSecond * robotVelocity.vyMetersPerSecond);

    double adjustedRPM = baseRPM - (velocityComponent * 50.0);

    return Math.max(2000.0, Math.min(5000.0, adjustedRPM)); // Clamp to reasonable range
  }

  /**
   * Enables Move-and-Shoot mode.
   * Robot pose and velocity are automatically retrieved from the Drive subsystem.
   */
  public void enableMoveAndShoot() {
    this.moveAndShootEnabled = true;
  }

  /**
   * Disables Move-and-Shoot mode.
   */
  public void disableMoveAndShoot() {
    this.moveAndShootEnabled = false;
  }

  // ========== Flywheel Methods ==========

  /**
   * Sets the flywheel velocity setpoint.
   */
  public void setFlywheelVelocity(double velocityRotationsPerSec) {
    desiredFlywheelVelocity = velocityRotationsPerSec;
  }

  /**
   * Checks if the flywheel is at the setpoint velocity.
   */
  @AutoLogOutput(key = "Shooter/Flywheel/AtSetpoint")
  public boolean flywheelAtSetpoint() {
    if (desiredFlywheelVelocity == 0.0) {
      return true;
    }
    double error = Math.abs(flywheelInputs.leaderVelocityRotationsPerSec - desiredFlywheelVelocity);
    return error < FlywheelConstants.VELOCITY_TOLERANCE;
  }

  /**
   * Gets the current flywheel velocity.
   */
  @AutoLogOutput(key = "Shooter/Flywheel/Velocity")
  public double getFlywheelVelocity() {
    return flywheelInputs.leaderVelocityRotationsPerSec;
  }

  // ========== Hood Methods ==========

  /**
   * Sets the hood angle setpoint.
   */
  public void setHoodAngle(double angleRadians) {
    desiredHoodAngle = angleRadians;
  }

  /**
   * Sets the hood angle based on distance to target.
   */
  public void setHoodAngleFromDistance(double distanceMeters) {
    // Simple linear interpolation (should be replaced with lookup table)
    // TODO MAKE LOOKUP TABLE
    double minDistance = 1.0; // meters
    double maxDistance = 10.0; // meters
    double minAngle = HoodConstants.MIN_ANGLE_RAD;
    double maxAngle = HoodConstants.MAX_ANGLE_RAD;

    double normalizedDistance = (distanceMeters - minDistance) / (maxDistance - minDistance);
    normalizedDistance = Math.max(0.0, Math.min(1.0, normalizedDistance));

    double angle = minAngle + normalizedDistance * (maxAngle - minAngle);
    setHoodAngle(angle);
  }

  /**
   * Gets the current hood angle.
   * Returns angle in radians (0° = horizontal, positive = up).
   */
  @AutoLogOutput(key = "Shooter/Hood/CurrentAngle")
  public double getHoodCurrentAngle() {
    // Convert from 0-1 range back to angle in radians
    // Simulation: absolutePositionRotations is (angle - MIN) / (MAX - MIN)
    // So: angle = absolutePositionRotations * (MAX - MIN) + MIN
    double normalized = hoodInputs.absolutePositionRotations;
    double angle = normalized * (HoodConstants.MAX_ANGLE_RAD - HoodConstants.MIN_ANGLE_RAD)
        + HoodConstants.MIN_ANGLE_RAD;
    return angle;
  }

  /**
   * Checks if the hood is at the setpoint angle.
   */
  @AutoLogOutput(key = "Shooter/Hood/AtSetpoint")
  public boolean hoodAtSetpoint() {
    double error = Math.abs(getHoodCurrentAngle() - desiredHoodAngle);
    return error < HoodConstants.ANGLE_TOLERANCE;
  }

  // ========== Turret Methods ==========

  /**
   * Solves the Chinese Remainder Theorem to determine absolute multi-turn
   * position.
   */
  private double solveCRT(double turretEncoderValue, double hoodEncoderValue) {
    int n1 = TurretConstants.ABSOLUTE_ENCODER_TEETH; // 19
    int n2 = TurretConstants.HOOD_ENCODER_TEETH; // 21

    int a1 = (int) Math.round(turretEncoderValue * n1);
    int a2 = (int) Math.round(hoodEncoderValue * n2);

    int m1 = n2; // 21
    int m2 = n1; // 19
    int lcm = n1 * n2; // 399

    int inv1 = 10; // Pre-calculated: 21^(-1) mod 19 = 10
    int inv2 = 10; // Pre-calculated: 19^(-1) mod 21 = 10

    long x = ((long) a1 * m1 * inv1 + (long) a2 * m2 * inv2) % lcm;
    if (x < 0) {
      x += lcm;
    }

    return (double) x / lcm;
  }

  /**
   * Gets the current turret angle.
   * In simulation, uses the encoder value directly.
   * In real hardware, uses CRT for multi-turn positioning.
   * Returns angle relative to robot forward direction (0° = forward, positive =
   * CCW).
   */
  @AutoLogOutput(key = "Shooter/Turret/CurrentAngle")
  public double getTurretCurrentAngle() {
    // In simulation, the absolute encoder value is normalized to 0-1 range
    // where 0 = -π and 1 = π
    // So: angle = (value * 2π) - π
    double turretEncoderValue = turretInputs.absolutePositionRotations;

    // Clamp encoder value to valid range [0, 1] to prevent issues
    turretEncoderValue = Math.max(0.0, Math.min(1.0, turretEncoderValue));

    // Convert from 0-1 range to -π to π
    double angle = (turretEncoderValue * 2.0 * Math.PI) - Math.PI;

    // Normalize to [-π, π] range
    return MathUtil.inputModulus(angle, -Math.PI, Math.PI);
  }

  /**
   * Sets the turret angle setpoint.
   */
  public void setTurretAngle(double angleRadians) {
    desiredTurretAngle = MathUtil.inputModulus(angleRadians, TurretConstants.MIN_ANGLE_RAD,
        TurretConstants.MAX_ANGLE_RAD);
  }

  /**
   * Checks if the turret is at the setpoint angle.
   */
  @AutoLogOutput(key = "Shooter/Turret/AtSetpoint")
  public boolean turretAtSetpoint() {
    double currentAngle = getTurretCurrentAngle();
    double setpointAngle = desiredTurretAngle;

    // Calculate error with proper wrap-around handling
    double error = MathUtil.inputModulus(setpointAngle - currentAngle, -Math.PI, Math.PI);
    return Math.abs(error) < TurretConstants.ANGLE_TOLERANCE;
  }

  // ========== Combined Methods ==========

  /**
   * Checks if the shooter is ready to fire.
   * All components must be at their setpoints.
   */
  @AutoLogOutput(key = "Shooter/ReadyToFire")
  public boolean isReadyToFire() {
    return flywheelAtSetpoint() && hoodAtSetpoint() && turretAtSetpoint();
  }

  /**
   * Stops all shooter components.
   */
  public void stop() {
    desiredFlywheelVelocity = 0.0;
    desiredHoodAngle = getHoodCurrentAngle(); // Hold current position
    desiredTurretAngle = getTurretCurrentAngle(); // Hold current position
    flywheelIO.stop();
    hoodIO.stop();
    turretIO.stop();
  }

  public Command idleFlywheelCommand() {
    return Commands.run(() -> {
      this.setFlywheelVelocity(50);
    }, this);
  }

//   public Command autoScoreCommand(Intake intake, Kicker kicker) {
//     if (Constants.currentMode == Constants.Mode.SIM) {
//       return Commands.parallel(
//           Commands.run(() -> this.prep = false),
//           // Always keep aiming while the button is held
//           Commands.run(this::enableMoveAndShoot, this),

//           // Repeating sequence for the actual "shots"
//           Commands.repeatingSequence(
//               // 1. Wait until the shooter is physically ready
//               Commands.waitUntil(() -> true),

//               // 2. Fire the hardware/kicker and physics sim simultaneously
//               Commands.parallel(
//                   kicker.fireCommand().withTimeout(0.1), // Quick pulse of the kicker
//                   Commands.runOnce(() -> this.launchSimulatedFuel(intake))),

//               // 3. The "Stagger" delay (e.g., 0.1s = 10 balls per second)
//               Commands.waitSeconds(0.2)));
//     } else {
//       return Commands.parallel(
//           Commands.run(() -> {
//             // Coordinate aim uses robot pose to calculate heading to hub
//             this.prep = false;
//             this.enableMoveAndShoot();
//           }, this),
//           Commands.sequence(
//               Commands.waitUntil(this::isReadyToFire),
//               kicker.fireCommand()));
//     }
//   }

  public Command prepCommand() {
    return new InstantCommand(() -> {
      this.enableMoveAndShoot();
      this.prep = true;
    }, this);
  }

//   public Command shootforseconds(Intake intake, Kicker kicker, double seconds) {
//     return new ParallelCommandGroup(
//         new RepeatCommand(autoScoreCommand(intake, kicker)),
//         new WaitCommand(seconds)).withTimeout(seconds);
//   }

  // SIMULATION STUFF

//   private void launchSimulatedFuel(Intake intake) {
//     if (Constants.currentMode != Constants.Mode.SIM || driveTrainSimulation == null || !intake.decrementBall())
//       return;

//     // 1. Gather current robot state
//     var robotPose = driveTrainSimulation.getSimulatedDriveTrainPose();
//     ChassisSpeeds chassisSpeeds = driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

//     // 2. Calculate launch parameters
//     // We combine robot rotation + turret rotation for the total field-relative
//     // heading
//     Rotation2d totalHeader = robotPose.getRotation().plus(Rotation2d.fromRadians(this.getTurretCurrentAngle()));

//     GamePieceProjectile fuelProjectile = new GamePieceProjectile(
//         Constants.FUEL_INFO,
//         robotPose.getTranslation(),
//         new Translation2d(0.1, 0), // Shooter offset from robot center (meters)
//         chassisSpeeds, // Adds robot inertia to the ball
//         totalHeader,
//         Distance.ofBaseUnits(0.5, Meters), // Launch height (meters)
//         LinearVelocity.ofBaseUnits(
//             this.getFlywheelVelocity() / 4, MetersPerSecond), // Convert RPM to meters/sec (example scaling)
//         Angle.ofBaseUnits((Math.PI / 2) - ((Math.PI / 8) + this.getHoodCurrentAngle()), Radians) // Vertical launch
//                                                                                                  // angle
//     );

//     // 3. Optional: Configure scoring visualization
//     fuelProjectile.withProjectileTrajectoryDisplayCallBack(
//         (poses) -> Logger.recordOutput("Sim/FuelTrajectory", poses.toArray(new Pose3d[0])),
//         (poses) -> Logger.recordOutput("Sim/FuelTrajectoryMiss", poses.toArray(new Pose3d[0])));

//     fuelProjectile.enableBecomesGamePieceOnFieldAfterTouchGround();
//     // 4. Register with the arena
//     SimulatedArena.getInstance().addGamePieceProjectile(fuelProjectile);
//   }
}