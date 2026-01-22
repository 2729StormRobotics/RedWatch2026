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

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.kicker.Kicker;

/**
 * AutoScore command that coordinates Turret, Hood, and Flywheel,
 * then triggers Kicker when ready.
 */
public class AutoScore {
  
  /**
   * Creates the AutoScore command.
   * 
   * @param shooter Shooter subsystem
   * @param kicker Kicker subsystem
   * @param targetPose Target pose on the field
   * @param distanceMeters Distance to target in meters
   * @param headingRadians Heading to target in radians
   * @param rpm Required flywheel RPM
   * @return Command that scores when ready
   */
  public static Command create(
      Shooter shooter,
      Kicker kicker,
      Pose2d targetPose,
      double distanceMeters,
      double headingRadians,
      double rpm) {
    
    return Commands.sequence(
        // Set all shooter components to target values
        Commands.parallel(
            Commands.runOnce(() -> shooter.setTurretAngle(headingRadians)),
            Commands.runOnce(() -> shooter.setHoodAngleFromDistance(distanceMeters)),
            Commands.runOnce(() -> shooter.setFlywheelVelocity(rpm / 60.0)) // Convert RPM to RPS
        ),
        // Wait until ready to fire
        Commands.waitUntil(() -> shooter.isReadyToFire()),
        // Fire the kicker
        kicker.fireCommand()
    ).withName("AutoScore");
  }
}
