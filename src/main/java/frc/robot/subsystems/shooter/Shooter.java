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
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  /** Desired turret angle in degrees (robot‑relative, 0° = forward, CCW positive). */
  private double desiredTurretAngleDeg = 0.0;

  private boolean prep = true;

  // Move-and-Shoot state
  private boolean moveAndShootEnabled = false;
  private boolean depotAimModeEnabled = false;

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
  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO, Drive drive, SwerveDriveSimulation driveTrainSimulation) {
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
    SmartDashboard.putNumber("Hood/Position", hoodIO.getPosition());
    SmartDashboard.putNumber("Shooter/Hood/SetpointAngle", desiredHoodAngle);
    SmartDashboard.putBoolean("Shooter/Hood/AtSetpoint", hoodAtSetpoint());
    SmartDashboard.putNumber("Shooter/Hood/DesiredAngle", turretIO.getDesiredAngle());  

    if (moveAndShootEnabled) {
      if (depotAimModeEnabled) {
        updateDepotAim(prep);
      } else {
        updateMoveAndShoot(prep);
      }
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
    // hoodIO.setAngle(desiredHoodAngle);
    // 
    // Always update turret setpoint
     turretIO.setAngle(desiredTurretAngleDeg);

    // Log shooter state
    Logger.recordOutput("Shooter/ReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/MoveAndShootEnabled", moveAndShootEnabled);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", desiredHoodAngle);
    Logger.recordOutput("Shooter/Turret/SetpointAngleDeg", desiredTurretAngleDeg);

    // Update 3D mechanism visualization
    // Turret angle is robot-relative (0° = robot forward, positive = CCW)
    mechanism3d.setShooter(
        Rotation2d.fromDegrees(getTurretCurrentAngleDeg()),
        new Rotation2d(getHoodCurrentAngle()));
    // Log the mechanism poses
    mechanism3d.log();
  }

  /**
   * Shared aiming logic for a given field-relative target point.
   * Calculates heading to target and adjusts turret/hood/flywheel accordingly.
   */
  private void updateAimToTarget(Translation2d targetPoint, boolean isPrep) {
    // --- SIM-AWARE POSE RETRIEVAL ---
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

    // Calculate vector from robot to hub (in field coordinates)
    Translation2d robotToTarget = targetPoint.minus(robotPose.getTranslation());

    // Check if target is valid (non-zero distance)
    double distanceToHub = robotToTarget.getNorm();
    if (distanceToHub < 0.01) {
      disableMoveAndShoot();
      return;
    }

    // Calculate heading to hub in field coordinates
    Rotation2d headingToHub = robotToTarget.getAngle();

    // Calculate required turret angle relative to robot forward direction
    Rotation2d robotRotation = robotPose.getRotation();
    Rotation2d turretRotation = headingToHub.minus(robotRotation);

    // Normalize and clamp
    // double turretAngle = MathUtil.inputModulus(turretRotation.getRadians(), -Math.PI, Math.PI);
    // turretAngle = MathUtil.clamp(turretAngle, TurretConstants.MIN_ANGLE_RAD, TurretConstants.MAX_ANGLE_RAD);
    // setTurretAngle(turretAngle);

    // Calculate required flywheel RPM (now using field-relative velocity for better
    // compensation)
    double requiredRPM = calculateRequiredRPM(distanceToHub, robotVelocity);
    setFlywheelVelocity(requiredRPM / 60.0);
    setHoodAngleFromDistance(distanceToHub);

    // Log target information
    Logger.recordOutput("Shooter/isPrep", isPrep);
    Logger.recordOutput("Shooter/TargetDistance", distanceToHub);
    Logger.recordOutput("Shooter/TargetHeading", headingToHub.getDegrees());
    // Logger.recordOutput("Shooter/TurretSetpoint", Math.toDegrees(turretAngle));
    Logger.recordOutput("Shooter/SimPoseUsed", frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM);
  }

  /**
   * Updates Move-and-Shoot calculations targeting the hub.
   */
  private void updateMoveAndShoot(boolean isPrep) {
    // Get hub center position (alliance-relative)
    Translation2d hubCenter;

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    if (isFlipped) {
      hubCenter = FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    } else {
      hubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    }

    updateAimToTarget(hubCenter, isPrep);
  }

  /**
   * Updates aiming calculations targeting our depot (for passes to our side).
   * Uses alliance-flipped depot center so it always represents "our" depot.
   */
  private void updateDepotAim(boolean isPrep) {
    Translation2d depotCenter =
        frc.robot.util.drive.AllianceFlipUtil
            .apply(FieldConstants.Depot.depotCenter)
            .toTranslation2d();
    updateAimToTarget(depotCenter, isPrep);
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

  /**
   * Enables depot-aim mode (aims at our depot instead of the hub).
   * Also turns on move-and-shoot so pose-based aiming is active.
   */
  public void enableDepotAim() {
    this.depotAimModeEnabled = true;
    this.enableMoveAndShoot();
  }

  /**
   * Disables depot-aim mode. Does not automatically disable move-and-shoot,
   * so other aim modes (like hub aiming) can still be used.
   */
  public void disableDepotAim() {
    this.depotAimModeEnabled = false;
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
    hoodIO.setAngle(angleRadians);
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
    double normalized = hoodInputs.absolutePositionRotations;
    // Convert from 0-1 range back to angle in radians:
    //   normalized = (angle - MIN) / (MAX - MIN)
    //   angle      = normalized * (MAX - MIN) + MIN
    double angle =
        normalized * (HoodConstants.MAX_ANGLE_RAD - HoodConstants.MIN_ANGLE_RAD)
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
   * Gets the current turret angle in degrees.
   * Backed by {@code TurretIOInputs.absoluteAngleDeg}, which is populated by both
   * real hardware IO and simulation IO.
   */
  @AutoLogOutput(key = "Shooter/Turret/CurrentAngleDeg")
  public double getTurretCurrentAngleDeg() {
    return turretInputs.absoluteAngleDeg;
  }

  /**
   * Sets the turret angle setpoint (degrees).
   */
  public void setTurretAngleDegrees(double angleDegrees) {
    desiredTurretAngleDeg =
        MathUtil.inputModulus(angleDegrees, TurretConstants.MIN_ANGLE_DEG, TurretConstants.MAX_ANGLE_DEG);
  }

  /**
   * Checks if the turret is at the setpoint angle.
   */
  @AutoLogOutput(key = "Shooter/Turret/AtSetpoint")
  public boolean turretAtSetpoint() {
    double currentDeg = getTurretCurrentAngleDeg();
    double setpointDeg = desiredTurretAngleDeg;

    // Calculate error with proper wrap-around handling in degrees
    double errorDeg =
        MathUtil.inputModulus(setpointDeg - currentDeg, -180.0, 180.0);
    return Math.abs(errorDeg) < TurretConstants.ANGLE_TOLERANCE_DEG;
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
    // desiredHoodAngle = getHoodCurrentAngle(); // Hold current position
    // desiredTurretAngleDeg = getTurretCurrentAngleDeg(); // Hold current position
    flywheelIO.stop();
    // hoodIO.stop();
    // turretIO.stop();
  }

  public Command idleFlywheelCommand() {
    return Commands.run(() -> {
      this.setFlywheelVelocity(0);
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
//     Rotation2d totalHeader = robotPose.getRotation().plus(Rotation2d.fromDegrees(this.getTurretCurrentAngleDeg()));

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

    public Command runPositionCommand(double ticks) {
    return run(() -> hoodIO.setPosition(ticks)).withName("HoodPosition: " + ticks);
  }

  public Command runPositionCommandConstant(CommandXboxController m_operatorController) {
    SmartDashboard.putNumber("Hood/leftx", (m_operatorController.getLeftX()));
        SmartDashboard.putNumber("Hood/ly", (m_operatorController.getLeftY()));
        SmartDashboard.putNumber("Hood/rx", (m_operatorController.getRightX()));
        SmartDashboard.putNumber("Hood/ry", (m_operatorController.getRightY()));
        SmartDashboard.putNumber("Hood/rt", (m_operatorController.getRightTriggerAxis()));
    return new RepeatCommand(run(() -> hoodIO.setPosition(-(19+(-m_operatorController.getLeftY()*18)))).withName("HoodPosition: " + -(19+(-m_operatorController.getRightY()*18))));
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("HoodStop");
  }

  // public Command runHoodCommand(double joyStick) {
  //   return runOnce(() -> hoodIO.setPercent(joyStick * 0.5));
  // }
  // public Command reverseHoodCommand() {
  //   return runOnce(() -> hoodIO.setPercent(-0.05)).withName("HoodStop");
  // }

  // public Command stopHoodCommand() {
  //   return runOnce(() -> hoodIO.setPercent(0)).withName("HoodStop");


  public Command setTurretAngleDegreesCommand(double degrees){
    return runOnce(() -> setTurretAngleDegrees(degrees)).withName("HoodStop");
  }
  

}