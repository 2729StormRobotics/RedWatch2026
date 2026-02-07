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

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Intake subsystem.
 * Follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface IntakeIO {
  /**
   * Auto-logged inputs for the intake.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class IntakeIOInputs {
    /** Pivot motor position in rotations */
    public double pivotPositionRotations = 0.0;
    
    /** Pivot motor velocity in rotations per second */
    public double pivotVelocityRotationsPerSec = 0.0;
    
    /** Pivot motor applied voltage */
    public double pivotAppliedVolts = 0.0;
    
    /** Pivot motor current draw in amps */
    public double pivotCurrentAmps = 0.0;
    
    /** Roller motor applied voltage */
    public double rollerAppliedVolts = 0.0;
    
    /** Roller motor current draw in amps */
    public double rollerCurrentAmps = 0.0;
    
    /** Beam break sensor state (true = broken, game piece detected) */
    public boolean beamBreakTriggered = false;
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the pivot position setpoint.
   *
   * @param positionRotations Target position in rotations
   */
  public default void setPivotPosition(double positionRotations) {}

  /**
   * Sets the roller output percentage.
   *
   * @param percent Percent output (-1.0 to 1.0)
   */
  public default void setRollerPercent(double percent) {}

  /**
   * Sets the pivot to run at a specific voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public default void setPivotVoltage(double volts) {}

  /**
   * Stops both motors.
   */
  public default void stop() {}
  /**
   * for simulation only.
   */
  public default boolean decrementBall() {return false;}
}