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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem with pivot (NEO) and roller (Vortex) motors.
 * Implements "Touch it, Own it" auto-retraction using beam-break inputs.
 * Updated for WPILib 2026 stateless TrapezoidProfile.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  private double desiredPivotPosition = IntakeConstants.RETRACTED_POSITION;
  private double desiredRollerPercent = 0.0;
  private boolean autoRetractEnabled = true;

  // Track the current state of the motion profile
  private TrapezoidProfile.State m_lastState = new TrapezoidProfile.State(0.0, 0.0);
  
  // 2026 API: Constructor only takes constraints
  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          IntakeConstants.kMaxVelocity,
          IntakeConstants.kMaxAcceleration)
  );

  /**
   * Creates a new Intake subsystem.
   *
   * @param io The IO implementation (real hardware or simulation)
   */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    io.updateInputs(inputs);

    // Process inputs for logging
    Logger.processInputs("Intake", inputs);

    // "Touch it, Own it" auto-retraction logic
    if (autoRetractEnabled && inputs.beamBreakTriggered) {
      // Game piece detected - automatically retract
      retract();
    }

    // Apply desired pivot position using ProfiledPID
    TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredPivotPosition, 0.0);
    
    // 2026 API Fix: calculate(timeStep, current, goal)
    m_lastState = m_profile.calculate(0.020, m_lastState, goal);
    
    // Push the profile setpoint to hardware
    io.setPivotPosition(m_lastState.position);

    // Apply desired roller percent
    io.setRollerPercent(desiredRollerPercent);
  }

  /** Deploys the intake. */
  public void deploy() {
    desiredPivotPosition = IntakeConstants.DEPLOYED_POSITION;
  }

  /** Retracts the intake. */
  public void retract() {
    desiredPivotPosition = IntakeConstants.RETRACTED_POSITION;
    desiredRollerPercent = 0.0; // Stop roller when retracting
  }

  /** Sets the roller speed for intaking. */
  public void intake() {
    desiredRollerPercent = IntakeConstants.INTAKE_ROLLER_SPEED;
  }

  /** Sets the roller speed for ejecting. */
  public void eject() {
    desiredRollerPercent = IntakeConstants.EJECT_ROLLER_SPEED;
  }

  /** Stops the roller. */
  public void stopRoller() {
    desiredRollerPercent = 0.0;
  }

  /** Checks if the intake is deployed. */
  @AutoLogOutput(key = "Intake/IsDeployed")
  public boolean isDeployed() {
    double error = Math.abs(inputs.pivotPositionRotations - IntakeConstants.DEPLOYED_POSITION);
    return error < IntakeConstants.POSITION_TOLERANCE;
  }

  /** Checks if a game piece is detected (beam break triggered). */
  @AutoLogOutput(key = "Intake/GamePieceDetected")
  public boolean isGamePieceDetected() {
    return inputs.beamBreakTriggered;
  }

  /** Enables or disables auto-retraction. */
  public void setAutoRetractEnabled(boolean enabled) {
    this.autoRetractEnabled = enabled;
  }

  /** Stops all intake motors. */
  public void stop() {
    retract();
    io.stop();
  }

  /** Command to deploy the intake. */
  public Command deployCommand() {
    return Commands.runOnce(this::deploy, this);
  }

  /** Command to retract the intake. */
  public Command retractCommand() {
    return Commands.runOnce(this::retract, this);
  }

  /** Command to run the intake. */
  public Command intakeCommand() {
    return Commands.run(() -> {
      deploy();
      intake();
    }, this);
  }
}