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

package frc.robot.subsystems.samplemotor;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for a simple motor subsystem.
 * This follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface SampleMotorIO {
  /**
   * Auto-logged inputs for the sample motor.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class SampleMotorIOInputs {
    /** Motor position in rotations */
    public double positionRotations = 0.0;

    /** Motor velocity in rotations per second */
    public double velocityRotationsPerSec = 0.0;

    /** Applied voltage to the motor */
    public double appliedVolts = 0.0;

    /** Current draw of the motor in amps */
    public double[] currentAmps = new double[] {};

    /** Temperature of the motor in Celsius */
    public double temperatureCelsius = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(SampleMotorIOInputs inputs) {}

  /**
   * Sets the motor output voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the motor output percentage.
   *
   * @param percent Percent output (-1.0 to 1.0)
   */
  public default void setPercent(double percent) {}

  /**
   * Enables or disables brake mode on the motor.
   *
   * @param enable True to enable brake mode, false for coast mode
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets the current limit for the motor.
   *
   * @param amps Current limit in amps
   */
  public default void setCurrentLimit(int amps) {}
}

