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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of SampleMotorIO.
 * Uses WPILib's DCMotorSim for physics simulation with proper logging.
 */
public class SampleMotorIOSim implements SampleMotorIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  public SampleMotorIOSim() {
    // Create motor simulation using NEO motor model
    // DCMotorSim constructor: LinearSystem, DCMotor
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), SampleMotorConstants.MOI_KG_M2, SampleMotorConstants.GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(SampleMotorIOInputs inputs) {
    // Update simulation with timestep
    motorSim.update(LOOP_PERIOD_SECS);

    // Update inputs from simulation
    // Convert radians to rotations
    inputs.positionRotations = motorSim.getAngularPositionRad() / (2.0 * Math.PI);
    inputs.velocityRotationsPerSec = motorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(motorSim.getCurrentDrawAmps())};
    inputs.temperatureCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPercent(double percent) {
    setVoltage(percent * 12.0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Brake mode doesn't affect simulation significantly
  }

  @Override
  public void setCurrentLimit(int amps) {
    // Current limiting doesn't affect simulation significantly
  }
}

