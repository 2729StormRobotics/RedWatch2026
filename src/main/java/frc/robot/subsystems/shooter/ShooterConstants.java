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

/**
 * Constants for the Shooter subsystem including 3D transforms for mechanism visualization.
 */
public final class ShooterConstants {
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
