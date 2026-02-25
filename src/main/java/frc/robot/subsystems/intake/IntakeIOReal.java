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
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.SparkIdleModeTuner;
import frc.robot.util.misc.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of IntakeIO using SparkMax motor controllers.
 */
public class IntakeIOReal implements IntakeIO {
  private final SparkFlex pivotMotor;
  private final SparkFlex rollerMotor;
  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController;
  
  private double pivotPositionSetpoint = 0.0;

  // Tunable PID gains for intake pivot
  private final LoggedTunableNumber kP_tunable =
      new LoggedTunableNumber("Intake/Pivot/kP", kP);
  private final LoggedTunableNumber kI_tunable =
      new LoggedTunableNumber("Intake/Pivot/kI", kI);
  private final LoggedTunableNumber kD_tunable =
      new LoggedTunableNumber("Intake/Pivot/kD", kD);

  public IntakeIOReal() {
    // Create pivot motor
    pivotMotor = new SparkFlex(PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();
    
    // Create roller motor
    rollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
    
    // Create beam break sensor

    
    // Configure pivot motor
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS)
        .inverted(PIVOT_INVERTED)
        .voltageCompensation(12.0);
    pivotConfig
        .encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0 / 60.0); // Convert RPM to RPS
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
    ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotPositionRotations = value);
    ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocityRotationsPerSec = value);
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

    // Allow runtime brake/coast selection for intake pivot and roller.
    SparkIdleModeTuner.syncIdleMode(pivotMotor, "Intake/PivotBrake", IdleMode.kBrake);
    SparkIdleModeTuner.syncIdleMode(rollerMotor, "Intake/RollerBrake", IdleMode.kCoast);

    // PID tuning from Elastic / SmartDashboard when in tuning mode.
    if (Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          this.hashCode(),
          values -> {
            double p = values[0];
            double i = values[1];
            double d = values[2];

            SparkMaxConfig cfg = new SparkMaxConfig();
            cfg.closedLoop.pid(p, i, d);
            tryUntilOk(
                pivotMotor,
                5,
                () ->
                    pivotMotor.configure(
                        cfg,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
          },
          kP_tunable, kI_tunable, kD_tunable);
    }

    // Read beam break sensor (inverted because DigitalInput is normally true when not triggered)
  }

  @Override
  public void setPivotPosition(double positionTicks) {
    pivotPositionSetpoint = positionTicks;
    pivotController.setSetpoint(positionTicks, ControlType.kPosition);
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
    rollerMotor.set(0.0);
  }
}