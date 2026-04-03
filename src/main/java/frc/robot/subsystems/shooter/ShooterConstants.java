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

  private static final Double shooterSpeedBump = 1.5;
  static {
    // --- Flywheel Speed (Rotations per Second) vs Distance (Meters) ---
    shooterSpeedMap.put(1.74, 240.0);
    shooterSpeedMap.put(2.08, 250.0);
    shooterSpeedMap.put(2.46, 260.0);
    shooterSpeedMap.put(2.48, 270.0);
    shooterSpeedMap.put(2.86, 271.0);
    shooterSpeedMap.put(2.9, 273.0);
    shooterSpeedMap.put(3.10, 275.0);
    shooterSpeedMap.put(3.67, 275.0);
    shooterSpeedMap.put(3.75, 280.0);
    shooterSpeedMap.put(4.0, 285.0);
    shooterSpeedMap.put(5.0, 290.0);

    // --- Hood Position (Motor Rotations) vs Distance (Meters) --
    //2.8-
    //1.7
    //2.08
    //2.5
//2.75
//3.0
    hoodPositionMap.put(2.08, 1.9);
    hoodPositionMap.put(2.47, 5.8);
    hoodPositionMap.put(2.6, 8.6);
    hoodPositionMap.put(2.75, 9.1);
    hoodPositionMap.put(2.99, 11.7);
    hoodPositionMap.put(3.0, 13.1);
    hoodPositionMap.put(3.1, 15.1);
    hoodPositionMap.put(3.5, 19.8);

    // --- Time of Flight (Seconds) vs Distance (Meters) ---
    // TODO: TUNE THESE VALUES! Record a video at 60fps, count frames from launch to target.
    // (e.g., 30 frames at 60fps = 0.5 seconds).
    tofMap.put(1.7, 0.9);
    tofMap.put(2.08, 0.9);
    tofMap.put(2.5, 1.0);
    tofMap.put(2.8, 1.2);
    tofMap.put(2.75, 1.5);
    tofMap.put(3.00, 1.6);
    tofMap.put(3.8, 2.5);
  }

  public static double getShooterSpeedRpsForDistance(double distanceMeters) {
    // if (Double.isNaN(distanceMeters)) return 250.0;
    Double val = shooterSpeedMap.get(distanceMeters) + shooterSpeedBump;
    // System.out.println(val);
    return val != null ? val : 250.0;
  }

  public static double getHoodPositionRotationsForDistance(double distanceMeters) {
    // if (Double.isNaN(distanceMeters)) return 28.9;
    Double val = hoodPositionMap.get(distanceMeters);
    return val != null ? val : 18.9;
  }

  public static double getTimeOfFlightForDistance(double distanceMeters) {
    if (Double.isNaN(distanceMeters)) return 0.5;
    Double val = tofMap.get(distanceMeters);
    return val != null ? val : 0.5;
  }
}