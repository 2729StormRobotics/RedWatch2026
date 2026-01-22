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

package frc.robot.subsystems.intake;

import static frc.robot.Constants.ElectricalLayout.*;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the Intake subsystem.
 */
public final class IntakeConstants {
  /** CAN IDs */
  public static final int PIVOT_MOTOR_ID = INTAKE_PIVOT_ID;
  public static final int ROLLER_MOTOR_ID = INTAKE_ROLLER_ID;
  
  /** DIO port for beam break sensor */
  public static final int BEAM_BREAK_PORT = INTAKE_BEAM_BREAK_PORT;
  
  /** Current limit for NEO motors in amps */
  public static final int PIVOT_CURRENT_LIMIT_AMPS = 40;
  
  /** Current limit for NEO Vortex motors in amps */
  public static final int ROLLER_CURRENT_LIMIT_AMPS = 60;
  
  /** Whether the pivot motor is inverted */
  public static final boolean PIVOT_INVERTED = false;
  
  /** Whether the roller motor is inverted */
  public static final boolean ROLLER_INVERTED = false;
  
  /** ProfiledPID constants for pivot */
  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMaxVelocity = 2.0; // rotations per second
  public static final double kMaxAcceleration = 4.0; // rotations per second squared
  
  /** Deployed position in rotations */
  public static final double DEPLOYED_POSITION = 1.0;
  
  /** Retracted position in rotations */
  public static final double RETRACTED_POSITION = 0.0;
  
  /** Position tolerance for atSetpoint check (rotations) */
  public static final double POSITION_TOLERANCE = 0.05;
  
  /** Roller speed when intaking (percent output) */
  public static final double INTAKE_ROLLER_SPEED = 0.8;
  
  /** Roller speed when ejecting (percent output) */
  public static final double EJECT_ROLLER_SPEED = -0.8;
  
  /** Gear ratio for pivot (motor rotations per pivot rotation) */
  public static final double PIVOT_GEAR_RATIO = 1.0;
  
  /** Moment of inertia in kg*m^2 (for simulation) */
  public static final double PIVOT_MOI_KG_M2 = 0.01;
  
  /** Length of the pivot arm in meters (for simulation) */
  public static final double ARM_LENGTH_M = 0.4;
  
  /** Mass of the intake in kg (for simulation) */
  public static final double MASS_KG = 1.0;
  
  private IntakeConstants() {}
}
