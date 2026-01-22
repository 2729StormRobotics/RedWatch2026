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

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Kicker subsystem.
 */
public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}
  public default void setVoltage(double volts) {}
  public default void stop() {}
}
