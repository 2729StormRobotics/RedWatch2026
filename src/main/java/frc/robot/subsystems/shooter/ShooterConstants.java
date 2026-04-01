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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Constants for the Shooter subsystem, including lookup tables for aiming. */
public class ShooterConstants {

  private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  static {
    // --- Flywheel Speed (Rotations per Second) vs Distance (Meters) ---
    shooterSpeedMap.put(1.74, 340.0);
    shooterSpeedMap.put(2.08, 350.0);
    shooterSpeedMap.put(2.46, 340.0);
    shooterSpeedMap.put(2.48, 340.0);
    shooterSpeedMap.put(2.86, 350.0);
    shooterSpeedMap.put(2.9, 360.0);
    shooterSpeedMap.put(3.67, 380.0);
    shooterSpeedMap.put(3.75, 390.0);

    // --- Hood Position (Motor Rotations) vs Distance (Meters) ---
    hoodPositionMap.put(2.08, 28.9);
    hoodPositionMap.put(2.47, 28.8);
    hoodPositionMap.put(2.6, 27.6);
    hoodPositionMap.put(2.75, 29.1);
    hoodPositionMap.put(2.99, 33.3);
    hoodPositionMap.put(3.0, 29.1);
    hoodPositionMap.put(3.1, 27.1);
    hoodPositionMap.put(3.5, 36.8);

    // --- Time of Flight (Seconds) vs Distance (Meters) ---
    // TODO: TUNE THESE VALUES! Record a video at 60fps, count frames from launch to target.
    // (e.g., 30 frames at 60fps = 0.5 seconds).
    tofMap.put(1.74, 0.45);
    tofMap.put(2.08, 0.52);
    tofMap.put(2.46, 0.60);
    tofMap.put(2.86, 0.68);
    tofMap.put(3.67, 0.85);
    tofMap.put(4.00, 0.95);
  }

  public static double getShooterSpeedRpsForDistance(double distanceMeters) {
    if (Double.isNaN(distanceMeters)) return 250.0;
    Double val = shooterSpeedMap.get(distanceMeters);
    return val != null ? val : 250.0;
  }

  public static double getHoodPositionRotationsForDistance(double distanceMeters) {
    if (Double.isNaN(distanceMeters)) return 28.9;
    Double val = hoodPositionMap.get(distanceMeters);
    return val != null ? val : 48.9;
  }

  public static double getTimeOfFlightForDistance(double distanceMeters) {
    if (Double.isNaN(distanceMeters)) return 0.5;
    Double val = tofMap.get(distanceMeters);
    return val != null ? val : 0.5;
  }
}