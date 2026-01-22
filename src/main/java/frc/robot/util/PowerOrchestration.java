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

package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Power Orchestration system that manages power distribution across all 17 motors.
 * Scales down non-critical subsystems (Intake, Hopper, Climb) when voltage drops below 9.5V
 * to prioritize Drive and Flywheels.
 */
public class PowerOrchestration {
  private static PowerOrchestration instance;
  private final PowerDistribution pdh;
  
  // Power priority groups
  public enum PowerPriority {
    CRITICAL,    // Drive, Flywheels - never scaled
    NON_CRITICAL // Intake, Hopper, Climb - scaled when voltage low
  }
  
  // Voltage threshold for power management
  private static final double VOLTAGE_THRESHOLD = 9.5; // volts
  private static final double MIN_SCALE_FACTOR = 0.3; // Minimum scale factor (30% power)
  
  private double currentScaleFactor = 1.0;
  private double busVoltage = 12.0;
  
  private PowerOrchestration() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
  }
  
  /**
   * Gets the singleton instance of PowerOrchestration.
   * @return The PowerOrchestration instance
   */
  public static PowerOrchestration getInstance() {
    if (instance == null) {
      instance = new PowerOrchestration();
    }
    return instance;
  }
  
  /**
   * Updates power management calculations. Should be called periodically.
   */
  public void periodic() {
    busVoltage = pdh.getVoltage();
    
    // Calculate scale factor based on voltage
    if (busVoltage < VOLTAGE_THRESHOLD) {
      // Scale factor decreases as voltage drops
      // Linear scaling from 1.0 at 9.5V to MIN_SCALE_FACTOR at 8.0V
      double voltageDrop = VOLTAGE_THRESHOLD - busVoltage;
      double maxDrop = VOLTAGE_THRESHOLD - 8.0; // Assume 8.0V is minimum
      currentScaleFactor = Math.max(MIN_SCALE_FACTOR, 1.0 - (voltageDrop / maxDrop) * (1.0 - MIN_SCALE_FACTOR));
    } else {
      currentScaleFactor = 1.0;
    }
    
    // Log power management data
    Logger.recordOutput("PowerOrchestration/BusVoltage", busVoltage);
    Logger.recordOutput("PowerOrchestration/ScaleFactor", currentScaleFactor);
    Logger.recordOutput("PowerOrchestration/TotalCurrent", pdh.getTotalCurrent());
  }
  
  /**
   * Gets the scale factor for non-critical subsystems.
   * @return Scale factor (0.0 to 1.0)
   */
  @AutoLogOutput(key = "PowerOrchestration/NonCriticalScaleFactor")
  public double getNonCriticalScaleFactor() {
    return currentScaleFactor;
  }
  
  /**
   * Gets the scale factor for critical subsystems (always 1.0).
   * @return Always 1.0
   */
  @AutoLogOutput(key = "PowerOrchestration/CriticalScaleFactor")
  public double getCriticalScaleFactor() {
    return 1.0;
  }
  
  /**
   * Gets the current bus voltage.
   * @return Bus voltage in volts
   */
  @AutoLogOutput(key = "PowerOrchestration/BusVoltage")
  public double getBusVoltage() {
    return busVoltage;
  }
  
  /**
   * Checks if power management is active (voltage below threshold).
   * @return True if power management is scaling non-critical systems
   */
  @AutoLogOutput(key = "PowerOrchestration/IsPowerManaged")
  public boolean isPowerManaged() {
    return busVoltage < VOLTAGE_THRESHOLD;
  }
  
  /**
   * Gets the total current draw from the PDH.
   * @return Total current in amps
   */
  public double getTotalCurrent() {
    return pdh.getTotalCurrent();
  }
  
  /**
   * Gets the current draw for a specific channel.
   * @param channel PDH channel number
   * @return Current in amps
   */
  public double getChannelCurrent(int channel) {
    return pdh.getCurrent(channel);
  }
}
