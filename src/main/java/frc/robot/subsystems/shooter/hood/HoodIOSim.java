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

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of HoodIO.
 * Uses WPILib's SingleJointedArmSim for physics simulation.
 */
public class HoodIOSim implements HoodIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  private final SingleJointedArmSim armSim;
  private double appliedVolts = 0.0;
  private double angleSetpoint = 0.0;
  private double currentAngle = 0.0;

  public HoodIOSim() {
    // Create arm simulation using NEO 550 motor model
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            GEAR_RATIO,
            MOI_KG_M2,
            ARM_LENGTH_M,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            false, // Simulate gravity
            MIN_ANGLE_RAD); // Starting angle
    
    // Initialize current angle from simulation
    currentAngle = armSim.getAngleRads();
    // Initialize setpoint to current angle to prevent initial jump
    angleSetpoint = currentAngle;
    
    // Set initial voltage to 0 to ensure simulation is ready
    armSim.setInputVoltage(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Get current angle BEFORE updating (for control calculation)
    currentAngle = armSim.getAngleRads();
    
    // Calculate control voltage with damping to prevent oscillation
    double error = angleSetpoint - currentAngle;
    double kP = 10.0;
    double kD = 0.1; // Damping term to reduce oscillation
    double velocity = armSim.getVelocityRadPerSec();
    
    // No gravity feedforward since gravity is disabled
    appliedVolts = MathUtil.clamp(kP * error - kD * velocity, -12.0, 12.0);
    
    // Set voltage BEFORE updating simulation
    armSim.setInputVoltage(appliedVolts);
    
    // Update simulation (uses voltage we just set)
    armSim.update(LOOP_PERIOD_SECS);
    
    // Get updated angle after simulation step
    currentAngle = armSim.getAngleRads();

    // Update inputs
    // Convert angle to 0-1 range for absolute encoder
    // Clamp to valid range to prevent issues
    double angleNormalized = MathUtil.clamp(currentAngle, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    inputs.absolutePositionRotations = (angleNormalized - MIN_ANGLE_RAD) / (MAX_ANGLE_RAD - MIN_ANGLE_RAD);
    inputs.motorPositionRotations = currentAngle / (2.0 * Math.PI);
    inputs.motorVelocityRotationsPerSec = armSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void setAngle(double angleRadians) {
    angleSetpoint = MathUtil.clamp(angleRadians, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
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
