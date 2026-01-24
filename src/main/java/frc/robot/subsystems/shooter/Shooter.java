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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.AlphaMechanism3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
  private final FlywheelIOInputsAutoLogged flywheelInputs = 
      new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = 
      new HoodIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = 
      new TurretIOInputsAutoLogged();
  
  // State variables
  private double desiredFlywheelVelocity = 0.0;
  private double desiredHoodAngle = 0.0;
  private double desiredTurretAngle = 0.0;
  
  // Move-and-Shoot state
  private boolean moveAndShootEnabled = false;

  // 3D Mechanism visualization
  private final AlphaMechanism3d mechanism3d = AlphaMechanism3d.getMeasured();

  /**
   * Creates a new Shooter super-subsystem.
   *
   * @param flywheelIO Flywheel IO implementation
   * @param hoodIO Hood IO implementation
   * @param turretIO Turret IO implementation
   * @param drive Drive subsystem for robot pose
   */
  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO, Drive drive) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;
    this.drive = drive;
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

    // Apply desired setpoints
    if (desiredFlywheelVelocity != 0.0) {
      flywheelIO.setVelocity(desiredFlywheelVelocity);
    }
    
    // Always update hood setpoint
    hoodIO.setAngle(desiredHoodAngle);
    
    // Always update turret setpoint
    turretIO.setAngle(desiredTurretAngle);
    
    // Update Move-and-Shoot if enabled
    if (moveAndShootEnabled) {
      updateMoveAndShoot();
    }
    
    // Log shooter state
    Logger.recordOutput("Shooter/ReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/MoveAndShootEnabled", moveAndShootEnabled);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", desiredHoodAngle);
    Logger.recordOutput("Shooter/Turret/SetpointAngle", desiredTurretAngle);

    // Update 3D mechanism visualization
    // Turret angle is robot-relative (0° = robot forward, positive = CCW)
    mechanism3d.setTurretAngle(new Rotation2d(getTurretCurrentAngle()));
    // Hood angle is relative to ground (0° = horizontal, positive = up)
    mechanism3d.setHoodAngle(new Rotation2d(getHoodCurrentAngle()));
    // Log the mechanism poses
    mechanism3d.log("Shooter", drive);
  }

  /**
   * Updates Move-and-Shoot calculations based on robot pose and velocity.
   * Calculates heading to hub and adjusts turret/hood/flywheel accordingly.
   */
  private void updateMoveAndShoot() {
    // Get current robot pose
    Pose2d robotPose = drive.getPose();
    
    // Get hub center position (alliance-relative)
    Translation2d hubCenter;
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
      // Red alliance - use opposite hub -- TODO
      hubCenter = new Translation2d(
          FieldConstants.Hub.topCenterPoint.getX(),
          FieldConstants.Hub.topCenterPoint.getY());
    } else {
      // Blue alliance - use near hub
      hubCenter = new Translation2d(
          FieldConstants.Hub.topCenterPoint.getX(),
          FieldConstants.Hub.topCenterPoint.getY());
    }
    
    // Calculate vector from robot to hub (in field coordinates)
    Translation2d robotToHub = hubCenter.minus(robotPose.getTranslation());
    
    // Check if target is valid (non-zero distance)
    double distanceToHub = robotToHub.getNorm();
    if (distanceToHub < 0.01) {
      // Robot is at hub center - disable move-and-shoot
      disableMoveAndShoot();
      return;
    }
    
    // Calculate heading to hub in field coordinates
    Rotation2d headingToHub = robotToHub.getAngle();
    
    // Calculate required turret angle relative to robot forward direction
    // Turret angle = field heading to hub - robot rotation
    // This gives us the angle the turret needs to rotate relative to robot forward (0° = forward, positive = CCW)
    Rotation2d robotRotation = robotPose.getRotation();
    Rotation2d turretRotation = headingToHub.minus(robotRotation);
    
    // Normalize to [-π, π] range and clamp to turret limits
    double turretAngle = MathUtil.inputModulus(turretRotation.getRadians(), -Math.PI, Math.PI);
    turretAngle = MathUtil.clamp(turretAngle, TurretConstants.MIN_ANGLE_RAD, TurretConstants.MAX_ANGLE_RAD);
    setTurretAngle(turretAngle);
    
    // Get robot velocity for move-and-shoot compensation
    ChassisSpeeds robotVelocity = drive.getChassisSpeeds();
    
    // Calculate required flywheel RPM based on distance and velocity
    double requiredRPM = calculateRequiredRPM(distanceToHub, robotVelocity);
    setFlywheelVelocity(requiredRPM / 60.0); // Convert RPM to RPS
    
    // Calculate required hood angle based on distance
    setHoodAngleFromDistance(distanceToHub);
    
    // Log target information
    Logger.recordOutput("Shooter/TargetDistance", distanceToHub);
    Logger.recordOutput("Shooter/TargetHeading", headingToHub.getDegrees());
    Logger.recordOutput("Shooter/TurretSetpoint", Math.toDegrees(turretAngle));
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
    double angle = normalized * (HoodConstants.MAX_ANGLE_RAD - HoodConstants.MIN_ANGLE_RAD) + HoodConstants.MIN_ANGLE_RAD;
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
   * Solves the Chinese Remainder Theorem to determine absolute multi-turn position.
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
   * Returns angle relative to robot forward direction (0° = forward, positive = CCW).
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
    desiredTurretAngle = MathUtil.inputModulus(angleRadians, TurretConstants.MIN_ANGLE_RAD, TurretConstants.MAX_ANGLE_RAD);
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
}
