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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of FlywheelIO.
 * Uses WPILib's FlywheelSim for physics simulation.
 */
public class FlywheelIOSim implements FlywheelIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  private final FlywheelSim flywheelSim;
  private double appliedVolts = 0.0;
  private double velocitySetpoint = 0.0;
  private double currentVelocity = 0.0;
  private double positionRotations = 0.0;

  public FlywheelIOSim() {
    // Create flywheel simulation using NEO Vortex motor model
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(2), // 2 motors
                MOI_KG_M2,
                GEAR_RATIO),
            DCMotor.getNeoVortex(2));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Update simulation with timestep
    flywheelSim.update(LOOP_PERIOD_SECS);
    
    // Get current velocity from simulation
    currentVelocity = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI); // Convert to RPS
    
    // Simple velocity controller simulation
    // Only control if setpoint is non-zero or there's a significant error
    if (Math.abs(velocitySetpoint) > 0.001 || Math.abs(currentVelocity - velocitySetpoint) > 0.1) {
      double error = velocitySetpoint - currentVelocity;
      double kP = 0.1; // Simple P controller for simulation
      appliedVolts = MathUtil.clamp(kP * error + kF * velocitySetpoint, -12.0, 12.0);
      flywheelSim.setInputVoltage(appliedVolts);
    } else {
      // At setpoint or zero setpoint - stop
      appliedVolts = 0.0;
      flywheelSim.setInputVoltage(0.0);
    }

    // Update position by integrating velocity
    positionRotations += currentVelocity * LOOP_PERIOD_SECS;

    // Update inputs from simulation
    inputs.leaderPositionRotations = positionRotations;
    inputs.leaderVelocityRotationsPerSec = currentVelocity;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps() / 2.0); // Split between two motors
    inputs.leaderTemperatureCelsius = 25.0;
    
    // Follower mirrors leader
    inputs.followerPositionRotations = inputs.leaderPositionRotations;
    inputs.followerVelocityRotationsPerSec = inputs.leaderVelocityRotationsPerSec;
    inputs.followerAppliedVolts = inputs.leaderAppliedVolts;
    inputs.followerCurrentAmps = inputs.leaderCurrentAmps;
    inputs.followerTemperatureCelsius = inputs.leaderTemperatureCelsius;
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    velocitySetpoint = velocityRotationsPerSec;
  }

  @Override
  public void setVoltage(double volts) {
    velocitySetpoint = 0.0;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    velocitySetpoint = 0.0;
    appliedVolts = 0.0;
    flywheelSim.setInputVoltage(0.0);
  }
}
