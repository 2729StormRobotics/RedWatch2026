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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of TurretIO.
 * Uses WPILib's SingleJointedArmSim for physics simulation.
 */
public class TurretIOSim implements TurretIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  private final SingleJointedArmSim armSim;
  private double appliedVolts = 0.0;
  private double angleSetpoint = 0.0;
  private double currentAngle = 0.0;

  public TurretIOSim() {
    // Create arm simulation using NEO motor model
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            GEAR_RATIO,
            MOI_KG_M2,
            ARM_LENGTH_M,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            false, // No gravity for turret (horizontal rotation)
            0.0); // Start at 0 (robot forward)
    
    // Initialize current angle from simulation
    currentAngle = 0.0;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Update simulation
    armSim.update(LOOP_PERIOD_SECS);
    
    // Get current angle from simulation
    // The simulation returns angle in the range [MIN_ANGLE_RAD, MAX_ANGLE_RAD]
    // which is [-π, π] for turret, but it might go outside this range
    double rawAngle = armSim.getAngleRads();
    
    // Normalize to [-π, π] range immediately to prevent wrap-around issues
    currentAngle = MathUtil.inputModulus(rawAngle, -Math.PI, Math.PI);
    
    // Normalize setpoint to [-π, π] for error calculation
    double normalizedSetpoint = MathUtil.inputModulus(angleSetpoint, -Math.PI, Math.PI);
    
    // Calculate error with wrap-around handling (shortest path)
    double error = MathUtil.inputModulus(normalizedSetpoint - currentAngle, -Math.PI, Math.PI);
    
    // Apply control voltage with damping to prevent oscillation
    double kP = 2.0;
    double kD = 0.2; // Damping term to reduce oscillation
    double velocity = armSim.getVelocityRadPerSec();
    appliedVolts = MathUtil.clamp(kP * error - kD * velocity, -12.0, 12.0);
    armSim.setInputVoltage(appliedVolts);

    // Update inputs
    // Convert angle to 0-1 range for absolute encoder (matching real encoder)
    // Real encoder: 0 = -π, 0.5 = 0, 1 = π
    // Use normalized current angle (already in [-π, π])
    inputs.absolutePositionRotations = (currentAngle + Math.PI) / (2.0 * Math.PI);
    // Clamp to ensure it's in [0, 1] range (should always be, but be safe)
    inputs.absolutePositionRotations = Math.max(0.0, Math.min(1.0, inputs.absolutePositionRotations));
    inputs.motorPositionRotations = currentAngle / (2.0 * Math.PI);
    inputs.motorVelocityRotationsPerSec = armSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void setAngle(double angleRadians) {
    angleSetpoint = MathUtil.inputModulus(angleRadians, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
  }

  @Override
  public void setVoltage(double volts) {
    angleSetpoint = 0.0;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    armSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    // Hold current position instead of going to zero
    angleSetpoint = currentAngle;
    appliedVolts = 0.0;
    armSim.setInputVoltage(0.0);
  }
}
