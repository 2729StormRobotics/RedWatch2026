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

import static frc.robot.Constants.ElectricalLayout.SAMPLE_MOTOR_CAN_ID;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Real hardware implementation of SampleMotorIO using a SparkMax motor controller.
 * This implementation logs all motor data through AdvantageKit.
 */
public class SampleMotorIOReal implements SampleMotorIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public SampleMotorIOReal() {
    motor = new SparkMax(SAMPLE_MOTOR_CAN_ID, MotorType.kBrushless);

    // Configure motor settings
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SampleMotorConstants.CURRENT_LIMIT_AMPS)
        .inverted(SampleMotorConstants.MOTOR_INVERTED);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor.getEncoder();

    // Configure encoder settings - conversion factors are set via encoder methods
    // Note: Encoder conversion factors are typically set through the encoder's configuration
    // For now, we'll use the default encoder units (rotations)

    // Reset encoder position
    encoder.setPosition(0.0);
  }

  @Override
  public void updateInputs(SampleMotorIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRotationsPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.temperatureCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setCurrentLimit(int amps) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(amps);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

