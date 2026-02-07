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

package frc.robot.subsystems.shooter.hood;

import static frc.robot.Constants.ElectricalLayout.*;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the Hood subsystem.
 */
public final class HoodConstants {
  /** CAN ID */
  public static final int MOTOR_ID = 11;
  
  /** Current limit for NEO 550 motors in amps */
  public static final int CURRENT_LIMIT_AMPS = 25;
  
  /** Whether the motor is inverted */
  public static final boolean MOTOR_INVERTED = false;
  
  /** Absolute encoder gear teeth (21T) */
  public static final int ABSOLUTE_ENCODER_TEETH = 21;
  
  /** Position PID constants */
  public static final double kP = 0.03;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  
  /** Angle tolerance for atSetpoint check (radians) */
  public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0);
  
  /** Minimum hood angle in radians */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(0.0);
  
  /** Maximum hood angle in radians */
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(30.0);
  
  /** Gear ratio (motor rotations per hood rotation) */
  public static final double GEAR_RATIO = 25.0;
  
  /** Moment of inertia in kg*m^2 (for simulation) */
  public static final double MOI_KG_M2 = 0.01;
  
  /** Length of the hood arm in meters (for simulation) */
  public static final double ARM_LENGTH_M = 0.3;
  
  /** Mass of the hood in kg (for simulation) */
  public static final double MASS_KG = 0.5;
  // private final int CURRENT_LIMIT_AMPS = 40; 

  private double positionSetpointRotations = 0.0;
  
  private HoodConstants() {}
}