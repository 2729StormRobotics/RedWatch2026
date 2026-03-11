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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.shooter.hood.HoodConstants;

/**
 * Constants for the Shooter subsystem including 3D transforms for mechanism visualization
 * and distance-based lookup table for shooter speed and hood angle.
 */
public final class ShooterConstants {

  // ========== Lookup table: distance from hub → shooter speed (RPS) and hood (motor rotations) ==========
  /** One row in the shooter lookup table. */
  public record LookupTableEntry(
      /** Distance from hub in meters. */
      double distanceFromHubMeters,
      /** Flywheel speed in rotations per second. */
      double shooterSpeedRps,
      /** Hood position in motor rotations (same units as HoodConstants.MIN/MAX_POSITION_ROTATIONS). */
      double hoodPositionRotations) {}

  /**
   * Lookup table: (distance from hub, shooter speed RPS, hood motor rotations).
   * Fill in with real data from characterization; entries should be ordered by distance ascending.
   */
  public static final LookupTableEntry[] LOOKUP_TABLE_HOOD = {
    new LookupTableEntry(2.08, 48.9, 4.8),
    new LookupTableEntry(2.47, 48.8, 20.8),
    new LookupTableEntry(2.6,47.6, 16.976),
    new LookupTableEntry(2.75,49.1, 16.8),
    new LookupTableEntry(2.99, 53.3, 5.9),
    new LookupTableEntry(3.0,49.1, 16.8),
    new LookupTableEntry(3.1,47.1, 22),
    new LookupTableEntry(3.5,56.8, 10.15),
  };

  public static final LookupTableEntry[] LOOKUP_TABLE_SHOOTER = {
    new LookupTableEntry(1.74, 240, 22.7),
    new LookupTableEntry(2.08, 250, 23.796),
    new LookupTableEntry(2.46,240, 22),
    new LookupTableEntry(2.48,240, 16.8),
    new LookupTableEntry(2.86,250, 16.89),
    new LookupTableEntry(2.9,260, 14),
    new LookupTableEntry(3.67, 280, 10.15),
    new LookupTableEntry(3.75,290, 10.15),
  };

  /**
   * Interpolates shooter speed (RPS) for a given distance using {@link #LOOKUP_TABLE}.
   * Clamps to first/last table value if distance is outside the table range.
   */
  public static double getShooterSpeedRpsForDistance(double distanceMeters) {
    if (LOOKUP_TABLE_SHOOTER.length == 0) return 0.0;
    if (distanceMeters <= LOOKUP_TABLE_SHOOTER[0].distanceFromHubMeters())
      return LOOKUP_TABLE_SHOOTER[0].shooterSpeedRps();
    if (distanceMeters >= LOOKUP_TABLE_SHOOTER[LOOKUP_TABLE_SHOOTER.length - 1].distanceFromHubMeters())
      return LOOKUP_TABLE_SHOOTER[LOOKUP_TABLE_SHOOTER.length - 1].shooterSpeedRps();
    for (int i = 0; i < LOOKUP_TABLE_SHOOTER.length - 1; i++) {
      double d0 = LOOKUP_TABLE_SHOOTER[i].distanceFromHubMeters();
      double d1 = LOOKUP_TABLE_SHOOTER[i + 1].distanceFromHubMeters();
      if (distanceMeters >= d0 && distanceMeters <= d1) {
        double t = (distanceMeters - d0) / (d1 - d0);
        return LOOKUP_TABLE_SHOOTER[i].shooterSpeedRps() + t * (LOOKUP_TABLE_SHOOTER[i + 1].shooterSpeedRps() - LOOKUP_TABLE_SHOOTER[i].shooterSpeedRps());
      }
    }
    return LOOKUP_TABLE_SHOOTER[LOOKUP_TABLE_SHOOTER.length - 1].shooterSpeedRps();
  }

  /**
   * Interpolates hood position (motor rotations) for a given distance using {@link #LOOKUP_TABLE}.
   * Clamps to first/last table value if distance is outside the table range.
   */
  public static double getHoodPositionRotationsForDistance(double distanceMeters) {
    if (LOOKUP_TABLE_HOOD.length == 0) return (HoodConstants.MIN_POSITION_ROTATIONS + HoodConstants.MAX_POSITION_ROTATIONS) / 2.0;
    if (distanceMeters <= LOOKUP_TABLE_HOOD[0].distanceFromHubMeters())
      return LOOKUP_TABLE_HOOD[0].hoodPositionRotations();
    if (distanceMeters >= LOOKUP_TABLE_HOOD[LOOKUP_TABLE_HOOD.length - 1].distanceFromHubMeters())
      return LOOKUP_TABLE_HOOD[LOOKUP_TABLE_HOOD.length - 1].hoodPositionRotations();
    for (int i = 0; i < LOOKUP_TABLE_HOOD.length - 1; i++) {
      double d0 = LOOKUP_TABLE_HOOD[i].distanceFromHubMeters();
      double d1 = LOOKUP_TABLE_HOOD[i + 1].distanceFromHubMeters();
      if (distanceMeters >= d0 && distanceMeters <= d1) {
        double t = (distanceMeters - d0) / (d1 - d0);
        return LOOKUP_TABLE_HOOD[i].hoodPositionRotations() + t * (LOOKUP_TABLE_HOOD[i + 1].hoodPositionRotations() - LOOKUP_TABLE_HOOD[i].hoodPositionRotations());
      }
    }
    return LOOKUP_TABLE_HOOD[LOOKUP_TABLE_HOOD.length - 1].hoodPositionRotations();
  }

  /** Transform from robot center to turret pivot point */
  public static final Transform3d robotToTurret =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(4.750), // X: forward/back (adjust based on your robot)
              Units.inchesToMeters(0), // Y: left/right (adjust based on your robot)
              Units.inchesToMeters(16.25)), // Z: height (adjust based on your robot)
          new Rotation3d(0.0, 0.0, -Math.PI/2));

  /** Transform from turret pivot to hood pivot point */
  public static final Transform3d turretToHood =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0.5), // X: forward from turret (0.105m)
              Units.inchesToMeters(4), // Y: left/right
              Units.inchesToMeters(4)), // Z: height (0.092m)
          new Rotation3d(0.0, 0.0, 0)); // Hood rotated 180° around Z

  private ShooterConstants() {}
}