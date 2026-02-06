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

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of HoodIO using logic from the Hood subsystem.
 * This implementation uses the internal relative encoder for position control.
 */
public class HoodIOReal implements HoodIO {
  // Hardcoded constants from the Hood subsystem
  private static final int MOTOR_ID = 12;
  private static final boolean INVERTED = false;
  private static final int CURRENT_LIMIT_AMPS = 40;
  private static final double kP = 0.03;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Soft limits from Hood subsystem
  private static final double FORWARD_SOFT_LIMIT = -1.0;
  private static final double REVERSE_SOFT_LIMIT = -37.0;

  private final SparkMax motor;
  private final RelativeEncoder internalEncoder;
  private final SparkClosedLoopController positionController;

  public HoodIOReal() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    internalEncoder = motor.getEncoder();
    positionController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(INVERTED)
        .voltageCompensation(12.0);

    // Soft limits configuration as requested
    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(FORWARD_SOFT_LIMIT)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(REVERSE_SOFT_LIMIT);

    // Maintain 1.0 factor to keep "rotations" unit consistent with Hood subsystem logic
    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Zero the encoder on startup as per the original subsystem logic
    tryUntilOk(motor, 5, () -> internalEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    ifOk(motor, internalEncoder::getPosition, (value) -> inputs.motorPositionRotations = value);
    ifOk(motor, internalEncoder::getVelocity, (value) -> inputs.motorVelocityRotationsPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
  }

  /**
   * Sets the target position in rotations.
   * Note: The input is clamped between the soft limits defined in the subsystem logic.
   */
  @Override
  public void setAngle(double targetRotations) {
    // Clamping to ensure we don't exceed soft limits even in code
    // Note: Forward limit is -1.0 (higher value) and Reverse is -37.0 (lower value)
    double clampedTarget = MathUtil.clamp(targetRotations, REVERSE_SOFT_LIMIT, FORWARD_SOFT_LIMIT);
    
    positionController.setReference(
        clampedTarget, 
        ControlType.kPosition, 
        ClosedLoopSlot.kSlot0
    );
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}