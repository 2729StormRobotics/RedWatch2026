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

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Flywheel subsystem.
 * Follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface FlywheelIO {
  /**
   * Auto-logged inputs for the flywheel.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class FlywheelIOInputs {
    /** Leader motor position in rotations */
    public double leaderPositionRotations = 0.0;
    
    /** Leader motor velocity in rotations per second */
    public double leaderVelocityRotationsPerSec = 0.0;
    
    /** Leader motor applied voltage */
    public double leaderAppliedVolts = 0.0;
    
    /** Leader motor current draw in amps */
    public double leaderCurrentAmps = 0.0;
    
    /** Follower motor position in rotations */
    public double followerPositionRotations = 0.0;
    
    /** Follower motor velocity in rotations per second */
    public double followerVelocityRotationsPerSec = 0.0;
    
    /** Follower motor applied voltage */
    public double followerAppliedVolts = 0.0;
    
    /** Follower motor current draw in amps */
    public double followerCurrentAmps = 0.0;
    
    /** Leader motor temperature in Celsius */
    public double leaderTemperatureCelsius = 0.0;
    
    /** Follower motor temperature in Celsius */
    public double followerTemperatureCelsius = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Sets the flywheel velocity setpoint.
   *
   * @param velocityRotationsPerSec Target velocity in rotations per second
   */
  public default void setVelocity(double velocityRotationsPerSec) {}

  /**
   * Sets the flywheel to run at a specific voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public default void setVoltage(double volts) {}

  /**
   * Stops the flywheel.
   */
  public default void stop() {}
}
