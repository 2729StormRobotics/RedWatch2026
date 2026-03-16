package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.SparkIdleModeTuner;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation for the Climb subsystem.
 * Configured with a Lead-Follower SparkMax setup.
 */
public class ClimbIOReal implements ClimbIO {
  private final SparkMax leader;
  // private final SparkMax follower;

  public ClimbIOReal() {
    leader = new SparkMax(LEADER_ID, MotorType.kBrushless);
    // follower = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // SparkMaxConfig followerConfig = new SparkMaxConfig();
    
    // 1. Configure Leader
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(LEADER_INVERTED)
        .voltageCompensation(12.0);
    
    // leaderConfig.softLimit
    //   .forwardSoftLimitEnabled(true)
    //   .forwardSoftLimit()
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit();

    leaderConfig.encoder.positionConversionFactor(1.0);
    
    leaderConfig.softLimit.forwardSoftLimit(0)
    .forwardSoftLimitEnabled(false)
    .reverseSoftLimit(-300)
    .reverseSoftLimitEnabled(false);
    // // 2. Configure Follower
    // followerConfig
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(CURRENT_LIMIT_AMPS)
    //     .follow(leader, FOLLOWER_INVERTED) // Follow leader with specific inversion
    //     .voltageCompensation(12.0);

    // Apply configurations
    tryUntilOk(
        leader, 
        5, 
        () -> leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // tryUntilOk(
    //     follower, 
    //     5, 
    //     () -> follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Leader Data
    ifOk(leader, new DoubleSupplier[] {leader::getAppliedOutput, leader::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leader, leader::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(leader, leader::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
    
    // Position calculation (assuming 0.01 meters per rotation)
    ifOk(leader, leader.getEncoder()::getPosition, (value) -> {
      inputs.positionMeters = value * 0.01; 
    });

    // Allow runtime brake/coast selection for both climb motors.
    SparkIdleModeTuner.syncIdleMode(leader, "Climb/LeaderBrake", IdleMode.kBrake);
    // SparkIdleModeTuner.syncIdleMode(follower, "Climb/FollowerBrake", IdleMode.kBrake);

    // Optional: Log follower current for health monitoring
    // ifOk(follower, follower::getOutputCurrent, (value) -> inputs.followerCurrentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
    // Follower automatically receives same signal via hardware follow mode
  }


  @Override
  public void setPercent(double percent) {
    leader.set(percent);
    // Follower automatically receives same signal via hardware follow mode
  }

  @Override
  public void stop() {
    leader.stopMotor();
    // follower.stopMotor(); // Explicitly stop both for safety
  }
}