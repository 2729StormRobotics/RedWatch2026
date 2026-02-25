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

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Turret subsystem.
 * Follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface TurretIO {
  /**
   * Auto-logged inputs for the turret.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class TurretIOInputs {
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

    public double absoluteEncoder19Pos;

    public double absoluteEncoder21Pos;

    public double absoluteAngleDeg;

    public double crtError;

    public double motorPositionDeg;

    public double motorVelocityDegPerSec;
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Sets the internal turret encoder position in degrees. */
  public default void setInternalPosition(double degrees) {}

  /**
   * Sets the turret angle setpoint.
   *
   * @param angleDegrees Target angle in degrees (robot‑relative, 0° = forward, CCW positive)
   */
  public default void setAngle(double angleDegrees) {}

  /**
   * Sets the turret to run at a specific voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public default void setVoltage(double volts) {}

  /**
   * Stops the turret.
   */
  public default void stop() {}
}