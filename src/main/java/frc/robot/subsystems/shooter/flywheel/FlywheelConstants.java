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

package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.ElectricalLayout.*;

/**
 * Constants for the Flywheel subsystem.
 */
public final class FlywheelConstants {
  /** CAN IDs */
  public static final int LEADER_MOTOR_ID = 9;
  public static final int FOLLOWER_MOTOR_ID = 10;
  
  /** Current limit for NEO Vortex motors in amps */
  public static final int CURRENT_LIMIT_AMPS = 60;
  
  /** Whether the leader motor is inverted */
  public static final boolean LEADER_INVERTED = false;
  
  /** Whether the follower motor is inverted */
  public static final boolean FOLLOWER_INVERTED = true;
  
  /** Velocity PID constants */
  public static final double kP = 0.0001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kF = 0.02; // Feedforward gain
  
  /** Velocity tolerance for atSetpoint check (rotations per second) */
  public static final double VELOCITY_TOLERANCE = 5.0;
  
  /** Maximum velocity in rotations per second */
  public static final double MAX_VELOCITY_RPS = 6000;
  
  /** Gear ratio (motor rotations per flywheel rotation) */
  public static final double GEAR_RATIO = 1.0;
  
  /** Moment of inertia in kg*m^2 (for simulation) */
  public static final double MOI_KG_M2 = 0.01;
  
  private FlywheelConstants() {}
}