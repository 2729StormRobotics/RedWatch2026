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

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Hood subsystem.
 * Follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface HoodIO {
  /**
   * Auto-logged inputs for the hood.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class HoodIOInputs {
    /** Absolute encoder position in rotations (0-1 range) */
    public double absolutePositionRotations = 0.0;
    
    /** Motor position in rotations */
    public double motorPositionRotations = 0.0;
    
    /** Motor velocity in rotations per second */
    public double motorVelocityRotationsPerSec = 0.0;
    
    /** Motor applied voltage */
    public double appliedVolts = 0.0;
    
    /** Motor current draw in amps */
    public double currentAmps = 0.0;
    
    /** Motor temperature in Celsius */
    public double temperatureCelsius = 0.0;

    
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Sets the hood angle setpoint.
   *
   * @param angleRadians Target angle in radians
   */
  public default void setAngle(double angleRadians) {}

  public default boolean isAtPosition(double target) {return true;}

  public default double getPosition() {return 0.0;}


  public default void setPosition(double targetRotations) {}
    public default void setPercent(double percebt) {}

  /**
   * Sets the hood to run at a specific voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public default void setVoltage(double volts) {}

  /**
   * Stops the hood.
   */
  public default void stop() {}
}