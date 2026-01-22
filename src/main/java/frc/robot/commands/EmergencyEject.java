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
import frc.robot.subsystems.kicker.Kicker;

/**
 * EmergencyEject command that reverses all game-piece handling motors.
 */
public class EmergencyEject {
  
  /**
   * Creates the EmergencyEject command.
   * 
   * @param intake Intake subsystem
   * @param hopper Hopper subsystem
   * @param kicker Kicker subsystem
   * @return Command that ejects game pieces
   */
  public static Command create(Intake intake, Hopper hopper, Kicker kicker) {
    return Commands.parallel(
        // Reverse intake roller
        Commands.run(() -> intake.eject(), intake),
        // Reverse hopper
        hopper.runCommand(-8.0), // Reverse at 8V
        // Reverse kicker
        Commands.run(() -> kicker.reverse(), kicker)
    ).withName("EmergencyEject");
  }
}
