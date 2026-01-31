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

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

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
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of IntakeIO using SparkMax motor controllers.
 */
public class IntakeIOReal implements IntakeIO {
  // Gear ratio constant to convert between Arm Rotations and Motor Rotations
  private static final double GEAR_RATIO = 13.333; 

  private final SparkMax pivotMotor;
  private final SparkMax rollerMotor;
  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController;
//   private final DigitalInput beamBreak;
  
  private double pivotPositionSetpoint = 0.0;

  public IntakeIOReal() {
    // Create pivot motor
    pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();
    
    // Create roller motor
    rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
    
    // Create beam break sensor
    // beamBreak = new DigitalInput(BEAM_BREAK_PORT);
    
    // Configure pivot motor
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS)
        .inverted(PIVOT_INVERTED)
        .voltageCompensation(12.0);
    pivotConfig
        .encoder
        // We keep the conversion factor at 1.0 so the motor controller "thinks" 
        // in Motor Rotations. We handle the gear ratio math in Java.
        .positionConversionFactor(1.0) 
        .velocityConversionFactor(1.0 / 60.0); // Convert RPM to RPS (Motor Shaft)
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);
    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Configure roller motor
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS)
        .inverted(ROLLER_INVERTED)
        .voltageCompensation(12.0);
    tryUntilOk(
        rollerMotor,
        5,
        () ->
            rollerMotor.configure(
                rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Reset encoder
    tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    
    // Convert Motor Rotations -> Mechanism Rotations for the logger
    ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotPositionRotations = value / GEAR_RATIO);
    
    // Convert Motor RPS -> Mechanism RPS
    ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocityRotationsPerSec = value / GEAR_RATIO);
    
    ifOk(
        pivotMotor,
        new DoubleSupplier[] {pivotMotor::getAppliedOutput, pivotMotor::getBusVoltage},
        (values) -> inputs.pivotAppliedVolts = values[0] * values[1]);
    ifOk(pivotMotor, pivotMotor::getOutputCurrent, (value) -> inputs.pivotCurrentAmps = value);
    
    sparkStickyFault = false;
    ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
    ifOk(rollerMotor, rollerMotor::getOutputCurrent, (value) -> inputs.rollerCurrentAmps = value);
    
    // Read beam break sensor (inverted because DigitalInput is normally true when not triggered)
    // inputs.beamBreakTriggered = !beamBreak.get();
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    pivotPositionSetpoint = positionRotations;
    
    // Convert Mechanism Rotations -> Motor Rotations
    // Example: 0.425 Mech Rotations (153 deg) * 13.333 Ratio = ~5.66 Motor Rotations
    double targetMotorRotations = positionRotations * GEAR_RATIO;
    
    pivotController.setSetpoint(targetMotorRotations, ControlType.kPosition);
  }

  @Override
  public void setRollerPercent(double percent) {
    rollerMotor.set(percent);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotPositionSetpoint = 0.0;
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    pivotPositionSetpoint = 0.0;
    pivotMotor.set(0.0);
    rollerMotor.set(0.0);
  }
}