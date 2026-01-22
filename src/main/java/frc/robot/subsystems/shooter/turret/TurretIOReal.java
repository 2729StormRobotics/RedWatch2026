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

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of TurretIO using SparkMax motor controller with absolute encoder.
 */
public class TurretIOReal implements TurretIO {
  private final SparkMax motor;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController positionController;
  
  private double angleSetpoint = 0.0;
  private double absoluteEncoderZeroOffset = 0.0; // Calibrated zero position

  public TurretIOReal() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    relativeEncoder = motor.getEncoder();
    absoluteEncoder = motor.getAbsoluteEncoder();
    positionController = motor.getClosedLoopController();
    
    // Configure motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(MOTOR_INVERTED)
        .voltageCompensation(12.0);
    config
        .absoluteEncoder
        .inverted(false)
        .positionConversionFactor(2.0 * Math.PI) // Convert to radians
        .averageDepth(2);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(kP, kI, kD);
    config
        .encoder
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor(2.0 * Math.PI / 60.0); // Convert RPM to rad/s
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Calibrate absolute encoder zero position
    absoluteEncoderZeroOffset = 0.0;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, absoluteEncoder::getPosition, (value) -> inputs.absolutePositionRotations = value);
    ifOk(motor, relativeEncoder::getPosition, (value) -> inputs.motorPositionRotations = value);
    ifOk(motor, relativeEncoder::getVelocity, (value) -> inputs.motorVelocityRotationsPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
  }

  @Override
  public void setAngle(double angleRadians) {
    angleSetpoint = MathUtil.inputModulus(angleRadians, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    
    // Convert angle to absolute encoder position
    double absolutePosition = (angleSetpoint / (2.0 * Math.PI)) + absoluteEncoderZeroOffset;
    
    // Wrap to 0-1 range
    absolutePosition = absolutePosition % 1.0;
    if (absolutePosition < 0.0) {
      absolutePosition += 1.0;
    }
    
    positionController.setSetpoint(angleSetpoint, ControlType.kPosition);
  }

  @Override
  public void setVoltage(double volts) {
    angleSetpoint = 0.0;
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    angleSetpoint = 0.0;
    motor.set(0.0);
  }
}
