// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Drive constants for MAXSwerve with 14T pinion + NEO Vortex drive motors.
 *
 * <p>Module CAN IDs, gear ratios, absolute encoder offsets, current limits and PID/feedforward
 * gains are configured in {@code src/main/deploy/swerve} (YAGSL). Only values the rest of the
 * code needs (PathPlanner, speed limits, geometry) live here, so keep the two in agreement:
 * module locations, wheel diameter and the drive gear ratio appear in both places.
 */
public final class DriveConstants {

  // ═══════════════════════════════════════════════════════════════════════════
  // PHYSICAL ROBOT DIMENSIONS (must match modules/*.json "location", in inches there)
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
  // WHEEL AND GEARING (must match modules/physicalproperties.json)
  // ═══════════════════════════════════════════════════════════════════════════

  // Wheel diameter - MEASURE YOUR ACTUAL WHEELS! They wear down over time.
  // Stock MAXSwerve uses 3" billet wheels. Worn wheels may be ~2.9"
  public static final double wheelDiameterMeters = Units.inchesToMeters(2.8669);
  public static final double wheelRadiusMeters = wheelDiameterMeters / 2.0;

  // MAXSwerve 14T Drive Gear Ratio: (45 × 22) / (14 × 15) = 4.71:1
  public static final int drivingMotorPinionTeeth = 14;
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0); // 4.714285714

  // ═══════════════════════════════════════════════════════════════════════════
  // SPEED LIMITS
  // ═══════════════════════════════════════════════════════════════════════════

  // NEO Vortex free speed: 6784 RPM -> ~5.49 m/s at the wheel with this gearing and wheel size.
  // We use 5.5 m/s to leave headroom for motor control
  public static final double maxSpeedMetersPerSec = 5.5;

  // Max acceleration - increase for snappier response, decrease if wheels slip
  public static final double maxAccelerationMetersPerSecSq = 4.5;

  // Max angular velocity = maxSpeed / driveBaseRadius
  // Calculated automatically via getMaxAngularSpeedRadPerSec()

  // ═══════════════════════════════════════════════════════════════════════════
  // GYRO
  // ═══════════════════════════════════════════════════════════════════════════
  public static final int pigeonCanId = 20;

  // ═══════════════════════════════════════════════════════════════════════════
  // PATHPLANNER CONFIGURATION
  // ═══════════════════════════════════════════════════════════════════════════
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
  public static final int driveMotorCurrentLimit = 70; // Amps (match physicalproperties.json)

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
