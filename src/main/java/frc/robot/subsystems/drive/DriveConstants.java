// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * DriveConstants for MAXSwerve with 14T pinion + NEO Vortex drive motors.
 * Configured for MAXIMUM SPEED.
 */
public final class DriveConstants {

  // ═══════════════════════════════════════════════════════════════════════════
  // PHYSICAL ROBOT DIMENSIONS
  // ═══════════════════════════════════════════════════════════════════════════
  public static final double trackWidth = Units.inchesToMeters(24.5);
  public static final double wheelBase = Units.inchesToMeters(24.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),   // Front Left
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),  // Front Right
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),  // Back Left
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)  // Back Right
  };

  // ═══════════════════════════════════════════════════════════════════════════
  // MAXSwerve 14T GEARING & WHEEL
  // ═══════════════════════════════════════════════════════════════════════════
  
  // Wheel diameter - MEASURE YOUR ACTUAL WHEELS! They wear down over time.
  // Stock MAXSwerve uses 3" billet wheels. Worn wheels may be ~2.9"
  public static final double wheelDiameterMeters = Units.inchesToMeters(2.8669);
//   public static final double wheelDiameterMeters = 0.038;
  public static final double wheelRadiusMeters = wheelDiameterMeters / 2.0;
  public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;

  // MAXSwerve 14T Drive Gear Ratio: (45 × 22) / (14 × 15) = 4.71:1
  public static final int drivingMotorPinionTeeth = 14;
  public static final double driveMotorReduction = 
      (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0); // 4.714285714

  // MAXSwerve Turn Gear Ratio: 9424:203 = 46.42:1
  public static final double turnMotorReduction = 9424.0 / 203.0;

  // ═══════════════════════════════════════════════════════════════════════════
  // SPEED LIMITS - CALCULATED FOR MAX PERFORMANCE
  // ═══════════════════════════════════════════════════════════════════════════
  
  // NEO Vortex free speed: 6784 RPM
  // Wheel speed = 6784 / 4.71 = 1440 RPM
  // Linear speed = 1440 × π × 0.0762m / 60 = 5.74 m/s (theoretical max)
  // We use 5.5 m/s to leave headroom for motor control
  public static final double maxSpeedMetersPerSec = 5.5;
  
  // Max acceleration - increase for snappier response, decrease if wheels slip
  public static final double maxAccelerationMetersPerSecSq = 4.5;
  
  // Max angular velocity = maxSpeed / driveBaseRadius
  // Calculated automatically via getMaxAngularSpeedRadPerSec()

  // ═══════════════════════════════════════════════════════════════════════════
  // ODOMETRY THREAD
  // ═══════════════════════════════════════════════════════════════════════════
  public static final double odometryFrequency = 100.0; // Hz

  // ═══════════════════════════════════════════════════════════════════════════
  // CAN IDs
  // ═══════════════════════════════════════════════════════════════════════════
  public static final int pigeonCanId = 20;

  // Drive motors (NEO Vortex on SparkFlex)
  public static final int frontLeftDriveCanId = 2;
  public static final int frontRightDriveCanId = 4;
  public static final int backLeftDriveCanId = 6;
  public static final int backRightDriveCanId = 8;

  // Turn motors (NEO 550 on SparkMax)
  public static final int frontLeftTurnCanId = 1;
  public static final int frontRightTurnCanId = 3;
  public static final int backLeftTurnCanId = 5;
  public static final int backRightTurnCanId = 7;

  // ═══════════════════════════════════════════════════════════════════════════
  // MODULE ZERO ROTATIONS (Absolute Encoder Offsets)
  // ═══════════════════════════════════════════════════════════════════════════
  // TODO: CALIBRATE THESE FOR YOUR ROBOT!
  // 1. Point all wheels forward (same direction)
  // 2. Read absolute encoder values
  // 3. Enter those values here
  public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(0);
  public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(Math.PI);
  public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(Math.PI);

  // ═══════════════════════════════════════════════════════════════════════════
  // DRIVE MOTOR CONFIGURATION (NEO Vortex)
  // ═══════════════════════════════════════════════════════════════════════════
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
  
  // Current limit - HIGHER = MORE TORQUE = FASTER ACCELERATION
  // NEO Vortex can handle 80A, but 60-70A is safer for longevity
  // Use 80A for competition if you need max acceleration
  public static final int driveMotorCurrentLimit = 70; // Amps

  // Encoder conversion factors (motor rotations → wheel radians)
  public static final double driveEncoderPositionFactor = 
      (2.0 * Math.PI) / driveMotorReduction; // radians per motor rotation
  public static final double driveEncoderVelocityFactor = 
      (2.0 * Math.PI) / 60.0 / driveMotorReduction; // rad/s per motor RPM

  // Drive PID (velocity control)
  // Keep P low to avoid oscillation - feedforward does most of the work
  public static final double driveKp = 0.0045;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.0075;

  // Drive Feedforward
  // kS: Voltage to overcome static friction (~0.1-0.2V typical)
  // kV: Voltage per rad/s = 12V / freeWheelSpeed(rad/s)
  //     Free wheel speed = 6784 RPM / 4.71 / 60 × 2π = 150.7 rad/s
  //     kV = 12 / 150.7 = 0.0796 ≈ 0.08
  public static final double driveKs = 0.19599;
  public static final double driveKv = 0.07793;

  // Simulation values
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.08;

  // ═══════════════════════════════════════════════════════════════════════════
  // TURN MOTOR CONFIGURATION (NEO 550)
  // ═══════════════════════════════════════════════════════════════════════════
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);
  public static final int turnMotorCurrentLimit = 20; // Amps
  public static final boolean turnInverted = false;

  // Turn encoder (absolute encoder outputs 0-1 rotations)
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2.0 * Math.PI; // rotations → radians
  public static final double turnEncoderVelocityFactor = (2.0 * Math.PI) / 60.0; // RPM → rad/s

  // Turn PID (position control with wrapping)
  // Higher P = snappier module rotation (but can oscillate if too high)
  public static final double turnKp = 1.0;
  public static final double turnKi = 0.0;
  public static final double turnKd = 0.0;
  public static final double turnPIDMinInput = 0.0;
  public static final double turnPIDMaxInput = 2.0 * Math.PI;

  // Simulation values
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;

  // ═══════════════════════════════════════════════════════════════════════════
  // PATHPLANNER CONFIGURATION
  // ═══════════════════════════════════════════════════════════════════════════
  public static final double robotMassKg = 18.1437; // ~40 lbs need to change
  public static final double robotMOI = 6.883;     // Moment of inertia (kg⋅m²)
  public static final double wheelCOF = 1.2;       // Coefficient of friction

  public static final RobotConfig ppConfig = new RobotConfig(
      robotMassKg,
      robotMOI,
      new ModuleConfig(
          wheelRadiusMeters,
          maxSpeedMetersPerSec,
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          driveMotorCurrentLimit,
          1),
      moduleTranslations);
}