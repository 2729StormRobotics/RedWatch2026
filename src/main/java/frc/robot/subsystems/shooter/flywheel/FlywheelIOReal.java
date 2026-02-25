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

package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

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
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.SparkIdleModeTuner;
import frc.robot.util.misc.LoggedTunableNumber;

/**
 * Real hardware implementation of FlywheelIO using SparkMax motor controllers.
 */
public class FlywheelIOReal implements FlywheelIO {
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder leaderEncoder;
  private final SparkClosedLoopController velocityController;
  
  private double velocitySetpoint = 0.0;

  // Tunable PID + FF gains (exposed to Elastic via SmartDashboard)
  private final LoggedTunableNumber kP_tunable =
      new LoggedTunableNumber("Shooter/Flywheel/kP", kP);
  private final LoggedTunableNumber kI_tunable =
      new LoggedTunableNumber("Shooter/Flywheel/kI", kI);
  private final LoggedTunableNumber kD_tunable =
      new LoggedTunableNumber("Shooter/Flywheel/kD", kD);
  private final LoggedTunableNumber kF_tunable =
      new LoggedTunableNumber("Shooter/Flywheel/kF", kF);

  // Local copy of feedforward gain so we can adjust it at runtime
  private double ffGain = kF;

  public FlywheelIOReal() {
    // Create leader motor
    leaderMotor = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
    leaderEncoder = leaderMotor.getEncoder();
    velocityController = leaderMotor.getClosedLoopController();
    
    // Create follower motor
    followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    
    // Configure leader motor
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(LEADER_INVERTED)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0 / 60.0); // Convert RPM to RPS
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);
    tryUntilOk(
        leaderMotor,
        5,
        () ->
            leaderMotor.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Configure follower motor
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(LEADER_MOTOR_ID, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(FOLLOWER_INVERTED)
        .voltageCompensation(12.0);
    tryUntilOk(
        followerMotor,
        5,
        () ->
            followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    
    // Reset encoder
    tryUntilOk(leaderMotor, 5, () -> leaderEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(leaderMotor, leaderEncoder::getPosition, (value) -> inputs.leaderPositionRotations = value);
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocityRotationsPerSec = value);
    ifOk(
        leaderMotor,
        new DoubleSupplier[] {leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage},
        (values) -> inputs.leaderAppliedVolts = values[0] * values[1]);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
    ifOk(leaderMotor, leaderMotor::getMotorTemperature, (value) -> inputs.leaderTemperatureCelsius = value);
    
    sparkStickyFault = false;
    ifOk(followerMotor, followerMotor.getEncoder()::getPosition, (value) -> inputs.followerPositionRotations = value);
    ifOk(followerMotor, followerMotor.getEncoder()::getVelocity, (value) -> inputs.followerVelocityRotationsPerSec = value);
    ifOk(
        followerMotor,
        new DoubleSupplier[] {followerMotor::getAppliedOutput, followerMotor::getBusVoltage},
        (values) -> inputs.followerAppliedVolts = values[0] * values[1]);
    ifOk(followerMotor, followerMotor::getOutputCurrent, (value) -> inputs.followerCurrentAmps = value);
    ifOk(followerMotor, followerMotor::getMotorTemperature, (value) -> inputs.followerTemperatureCelsius = value);

    // Allow runtime brake/coast selection for both flywheel motors.
    SparkIdleModeTuner.syncIdleMode(leaderMotor, "Shooter/FlywheelLeaderBrake", IdleMode.kCoast);
    SparkIdleModeTuner.syncIdleMode(followerMotor, "Shooter/FlywheelFollowerBrake", IdleMode.kCoast);

    // PID tuning from Elastic/SmartDashboard when in tuning mode.
    if (Constants.tuningMode) {
      // Only touch the config when a value actually changes to avoid extra CAN traffic.
      LoggedTunableNumber.ifChanged(
          this.hashCode(),
          values -> {
            double p = values[0];
            double i = values[1];
            double d = values[2];

            SparkMaxConfig cfg = new SparkMaxConfig();
            cfg.closedLoop.pid(p, i, d);
            tryUntilOk(
                leaderMotor,
                5,
                () ->
                    leaderMotor.configure(
                        cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
          },
          kP_tunable, kI_tunable, kD_tunable);

      // Feedforward gain (used when commanding velocity)
      kF_tunable.runUpdate(v -> ffGain = v);
    }
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    velocitySetpoint = velocityRotationsPerSec;
    double ffVolts = ffGain * velocityRotationsPerSec;
    velocityController.setSetpoint(
        velocityRotationsPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    velocitySetpoint = 0.0;
    leaderMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    velocitySetpoint = 0.0;
    leaderMotor.set(0.0);
  }
}