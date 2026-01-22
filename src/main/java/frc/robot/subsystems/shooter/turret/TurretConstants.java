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

package frc.robot.subsystems.shooter.turret;

import static frc.robot.Constants.ElectricalLayout.*;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the Turret subsystem.
 */
public final class TurretConstants {
  /** CAN ID */
  public static final int MOTOR_ID = TURRET_MOTOR_ID;
  
  /** Current limit for NEO motors in amps */
  public static final int CURRENT_LIMIT_AMPS = 40;
  
  /** Whether the motor is inverted */
  public static final boolean MOTOR_INVERTED = false;
  
  /** Absolute encoder gear teeth (19T) */
  public static final int ABSOLUTE_ENCODER_TEETH = 19;
  
  /** Hood absolute encoder gear teeth (21T) - used for CRT */
  public static final int HOOD_ENCODER_TEETH = 21;
  
  /** Position PID constants */
  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  
  /** Angle tolerance for atSetpoint check (radians) */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0);
  
  /** Minimum turret angle in radians (typically -180 to +180 degrees) */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-180.0);
  
  /** Maximum turret angle in radians */
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(180.0);
  
  /** Gear ratio (motor rotations per turret rotation) */
  public static final double GEAR_RATIO = 1.0;
  
  /** Moment of inertia in kg*m^2 (for simulation) */
  public static final double MOI_KG_M2 = 0.01;
  
  /** Length of the turret arm in meters (for simulation) */
  public static final double ARM_LENGTH_M = 0.2;
  
  /** Mass of the turret in kg (for simulation) */
  public static final double MASS_KG = 1.0;
  
  private TurretConstants() {}
}
