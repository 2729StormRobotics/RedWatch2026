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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * 3D mechanism visualization for the shooter subsystem.
 * Based on Mechanical Advantage's AlphaMechanism3d pattern.
 */
public class AlphaMechanism3d {
  private static AlphaMechanism3d measured;

  public static AlphaMechanism3d getMeasured() {
    if (measured == null) {
      measured = new AlphaMechanism3d();
    }
    return measured;
  }

  private Rotation2d turretAngle = Rotation2d.kZero; // Robot-relative
  private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground

  public void setTurretAngle(Rotation2d angle) {
    this.turretAngle = angle;
  }

  public void setHoodAngle(Rotation2d angle) {
    this.hoodAngle = angle;
  }

  /**
   * Log the component poses in field coordinates.
   * The poses are transformed from robot-relative to field-relative so they move with the robot.
   * Based on Mechanical Advantage's AlphaMechanism3d pattern.
   *
   * @param key The logging key prefix
   * @param drive Drive subsystem to get robot pose
   */
  public void log(String key, Drive drive) {
    // Get robot pose in field coordinates
    Pose2d robotPose2d = drive.getPose();
    Pose3d robotPose = new Pose3d(robotPose2d);

    // Build turret pose: robot -> turret base -> turret with rotation
    // Start from robot pose, apply robotToTurret transform, then apply turret rotation
    Pose3d turretPose = robotPose
        .transformBy(ShooterConstants.robotToTurret)
        .transformBy(new Transform3d(
            Translation3d.kZero,
            new Rotation3d(0.0, 0.0, turretAngle.getRadians())));

    // Build hood pose: turret -> hood (with rotation)
    Pose3d hoodPose = turretPose.transformBy(
        new Transform3d(
            ShooterConstants.turretToHood.getTranslation(),
            new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));

    // Log component poses as array (in field coordinates)
    Logger.recordOutput(key + "/Components", new Pose3d[] {turretPose, hoodPose});
    
  }
}
