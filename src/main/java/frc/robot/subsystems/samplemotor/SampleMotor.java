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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Sample motor subsystem demonstrating proper AdvantageKit logging patterns.
 * This subsystem can be used as a template for creating new subsystems.
 */
public class SampleMotor extends SubsystemBase {
  private final SampleMotorIO io;
  private final SampleMotorIOInputsAutoLogged inputs = new SampleMotorIOInputsAutoLogged();

  /** Desired output voltage for the motor */
  private double desiredVoltage = 0.0;

  /** Desired output percent for the motor */
  private double desiredPercent = 0.0;

  /**
   * Creates a new SampleMotor subsystem.
   *
   * @param io The IO implementation (real hardware or simulation)
   */
  public SampleMotor(SampleMotorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    io.updateInputs(inputs);

    // Process inputs for logging (this automatically logs all @AutoLog fields)
    Logger.processInputs("SampleMotor", inputs);

    // Apply desired output
    if (desiredVoltage != 0.0) {
      io.setVoltage(desiredVoltage);
    } else {
      io.setPercent(desiredPercent);
    }

    // Log additional outputs
    Logger.recordOutput("SampleMotor/DesiredVoltage", desiredVoltage);
    Logger.recordOutput("SampleMotor/DesiredPercent", desiredPercent);
  }

  /**
   * Sets the motor to run at a specific voltage.
   *
   * @param volts Voltage to apply (-12 to 12 volts)
   */
  public void setVoltage(double volts) {
    desiredVoltage = volts;
    desiredPercent = 0.0;
  }

  /**
   * Sets the motor to run at a specific percentage.
   *
   * @param percent Percent output (-1.0 to 1.0)
   */
  public void setPercent(double percent) {
    desiredPercent = percent;
    desiredVoltage = 0.0;
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    setPercent(0.0);
  }

  /**
   * Gets the current motor position in rotations.
   *
   * @return Motor position in rotations
   */
  @AutoLogOutput(key = "SampleMotor/PositionRotations")
  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * Gets the current motor velocity in rotations per second.
   *
   * @return Motor velocity in rotations per second
   */
  @AutoLogOutput(key = "SampleMotor/VelocityRotationsPerSec")
  public double getVelocity() {
    return inputs.velocityRotationsPerSec;
  }

  /**
   * Gets the current motor current draw in amps.
   *
   * @return Motor current in amps
   */
  @AutoLogOutput(key = "SampleMotor/CurrentAmps")
  public double getCurrent() {
    return inputs.currentAmps.length > 0 ? inputs.currentAmps[0] : 0.0;
  }

  /**
   * Gets the current motor temperature in Celsius.
   *
   * @return Motor temperature in Celsius
   */
  @AutoLogOutput(key = "SampleMotor/TemperatureCelsius")
  public double getTemperature() {
    return inputs.temperatureCelsius;
  }

  /**
   * Sets the brake mode for the motor.
   *
   * @param enable True to enable brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Creates a command to run the motor at a constant voltage.
   *
   * @param volts Voltage to apply
   * @return Command to run the motor
   */
  public Command runVoltage(double volts) {
    return Commands.run(() -> setVoltage(volts), this);
  }

  /**
   * Creates a command to run the motor at a constant percentage.
   *
   * @param percent Percent output (-1.0 to 1.0)
   * @return Command to run the motor
   */
  public Command runPercent(double percent) {
    return Commands.run(() -> setPercent(percent), this);
  }

  /**
   * Creates a command to stop the motor.
   *
   * @return Command to stop the motor
   */
  public Command stopCommand() {
    return Commands.runOnce(() -> stop(), this);
  }
}

