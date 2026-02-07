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

import static edu.wpi.first.units.Units.*;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of IntakeIO.
 * Uses WPILib's SingleJointedArmSim for physics simulation.
 */
public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  private final SingleJointedArmSim pivotSim;
  private final PIDController pivotController;
  private double pivotAppliedVolts = 0.0;
  private double pivotPositionSetpoint = 0.0;
  private double rollerAppliedVolts = 0.0;
  private boolean beamBreakTriggered = false;

  private final IntakeSimulation intakeSimulation;

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    // Create pivot arm simulation using NEO motor model
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            PIVOT_GEAR_RATIO,
            PIVOT_MOI_KG_M2,
            ARM_LENGTH_M,
            RETRACTED_POSITION * 2.0 * Math.PI, // Convert to radians
            DEPLOYED_POSITION * 2.0 * Math.PI,
            true, // Simulate gravity
            RETRACTED_POSITION * 2.0 * Math.PI); // Starting position
    
    pivotController = new PIDController(kP, kI, kD);

    this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Fuel",
        // Specify the drivetrain to which this intake is attached
        driveTrain,
        // Width of the intake
        Meters.of(0.7),
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update simulation with timestep
    pivotSim.update(LOOP_PERIOD_SECS);
    
    // Run pivot position controller
    if (pivotPositionSetpoint != 0.0 || Math.abs(pivotSim.getAngleRads() / (2.0 * Math.PI) - pivotPositionSetpoint) > 0.01) {
      double currentPosition = pivotSim.getAngleRads() / (2.0 * Math.PI);
      double output = pivotController.calculate(currentPosition, pivotPositionSetpoint);
      pivotAppliedVolts = MathUtil.clamp(output * 12.0, -12.0, 12.0);
      pivotSim.setInputVoltage(pivotAppliedVolts);
    }
    
    // Simulate roller (simple voltage model)
    // Roller simulation would use DCMotorSim, but for simplicity we'll just track voltage
    
    // Simulate beam break (triggered when intake is deployed and has been running)
    beamBreakTriggered = (pivotSim.getAngleRads() > DEPLOYED_POSITION * 0.8 * 2.0 * Math.PI) 
        && (Math.abs(rollerAppliedVolts) > 0.1);
    Logger.recordOutput("Intake/ballsCollected", intakeSimulation.getGamePiecesAmount());
    // Update inputs from simulation
    inputs.pivotPositionRotations = pivotSim.getAngleRads() / (2.0 * Math.PI);
    inputs.pivotVelocityRotationsPerSec = pivotSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerAppliedVolts / 12.0 * 10.0); // Simplified current calculation
    inputs.beamBreakTriggered = beamBreakTriggered;
  }

  @Override
  public boolean decrementBall(){
    return intakeSimulation.obtainGamePieceFromIntake();
  }
  @Override
  public void setPivotPosition(double positionRotations) {
    pivotPositionSetpoint = positionRotations;
    if (positionRotations > 0.2) {
      //deployed
      intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces

    } else {
      //retracted
      intakeSimulation.stopIntake();
    }
    
  }

  @Override
  public void setRollerPercent(double percent) {
    rollerAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotPositionSetpoint = 0.0;
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void stop() {
    pivotPositionSetpoint = 0.0;
    pivotAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    pivotSim.setInputVoltage(0.0);
  }
}