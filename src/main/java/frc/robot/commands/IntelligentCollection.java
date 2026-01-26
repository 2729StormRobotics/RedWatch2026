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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.hopper.Hopper;

/**
 * IntelligentCollection command that syncs Intake and Hopper,
 * automatically stopping when "Full" condition is detected.
 */
public class IntelligentCollection {
  
  /**
   * Creates the IntelligentCollection command.
   * 
   * @param intake Intake subsystem
   * @param hopper Hopper subsystem
   * @return Command that runs until full
   */
  public static Command create(Intake intake, Hopper hopper) {
    return Commands.sequence(
        // Deploy intake and start both systems
        Commands.parallel(
            intake.deployCommand(),
            Commands.waitSeconds(0.5), // Wait for deployment
            Commands.parallel(
                intake.intakeCommand(),
                hopper.runCommand(8.0) // Run hopper at 8V
            )
        ),
        
        // Stop both systems
        Commands.parallel(
            Commands.runOnce(() -> intake.stopRoller()),
            Commands.runOnce(() -> hopper.stop())
        )
    ).withName("IntelligentCollection");
  }
}
