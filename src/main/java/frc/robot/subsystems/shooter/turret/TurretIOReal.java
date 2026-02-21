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
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of TurretIO.
 * * This class contains the CRT logic for absolute positioning AND the 
 * RoboRIO-side ProfiledPIDController logic for smooth motion.
 */
public class TurretIOReal implements TurretIO {
  // Gear Constants
  private final double k_turretRingTeeth = 200.0;
  private final double k_gear19 = 19.0;
  private final double k_gear21 = 21.0;
  private final double k_gearboxRatio = 4.0; 

  // TUNE THESE OFFSETS BASED ON CALIBRATION
  private final double k_enc19Offset = 0.0;
  private final double k_enc21Offset = 0.0;

  // Hardware Objects
  private final SparkMax motor;
  private final SparkMax auxSpark;
  private final SparkAbsoluteEncoder encoder19;
  private final SparkAbsoluteEncoder encoder21;
  private final RelativeEncoder internalEncoder;

  // RIO-Side PID Controller with Motion Profiling
  // kP, kI, kD should be defined in TurretConstants
  private final TrapezoidProfile.Constraints m_constraints = 
      new TrapezoidProfile.Constraints(500.0, 600.0);
  private final ProfiledPIDController m_pidController = 
      new ProfiledPIDController(0.048, 0.0, 0.0, m_constraints);

  private double targetAngleDegrees = 0.0;
  private boolean isClosedLoop = false;
  private double lastAbsoluteAngleDeg = 0.0;

  /*
   * ===============================================================
   *  TURRET / HOOD ENCODER SAFETY BRING‑UP & TEST PLAN (REAL ROBOT)
   * ===============================================================
   *
   * GOAL: Prove that
   *   - encoder19 (turret) and encoder21 (hood on auxSpark) read correctly,
   *   - CRT angle is sane, and
   *   - closed‑loop motion never drives into hard stops unexpectedly.
   *
   * Only follow these steps on a real robot with at least one mentor watching.
   *
   * ---------------------------------------------------------------
   * PHASE 0 – MECHANICAL & WIRING CHECK (ROBOT DISABLED)
   * ---------------------------------------------------------------
   * 0.1  Confirm wiring:
   *      - Turret motor is CAN ID 12 (see TurretConstants.MOTOR_ID).
   *      - encoder19 is the absolute encoder on the turret motor SparkMax.
   *      - encoder21 is the 21T absolute encoder plugged into the HOOD SparkMax
   *        data port (auxSpark passed into this constructor).
   *
   * 0.2  Verify range and hard‑stops:
   *      - Manually rotate the turret by hand with robot disabled.
   *      - Confirm it cannot mechanically rotate beyond the expected range
   *        (± ~180° or whatever is safe for your design).
   *      - Make sure no cables can snag when rotating.
   *
   * ---------------------------------------------------------------
   * PHASE 1 – SENSOR SANITY CHECK (NO MOTOR MOTION)
   * ---------------------------------------------------------------
   * 1.1  Put the robot on blocks or otherwise ensure it CANNOT drive or shoot.
   *
   * 1.2  Enable robot with all turret commands idle (no commands should be
   *      calling setAngle / setVoltage yet).
   *
   * 1.3  On AdvantageScope / Shuffleboard, watch these logged values:
   *      - Shooter/Turret/Inputs/absoluteEncoder19Pos
   *      - Shooter/Turret/Inputs/absoluteEncoder21Pos
   *      - Shooter/Turret/Inputs/absoluteAngleDeg
   *      - Shooter/Turret/Inputs/crtError
   *
   * 1.4  Slowly move the turret BY HAND (if safe) a few degrees each way
   *      while robot is ENABLED but with motors commanded to 0:
   *      - absoluteEncoder19Pos should change smoothly in [0, 1).
   *      - absoluteEncoder21Pos should also change smoothly in [0, 1).
   *      - absoluteAngleDeg (CRT output) should change roughly linearly with
   *        the physical turret rotation.
   *      - crtError should stay small (well below 0.1). If it is large or
   *        jumps, STOP and fix encoder offsets (k_enc19Offset / k_enc21Offset)
   *        before proceeding.
   *
   * ---------------------------------------------------------------
   * PHASE 2 – OPEN‑LOOP MOTOR TEST (VERY LOW VOLTAGE)
   * ---------------------------------------------------------------
   * 2.1  Still on blocks, create a simple test command that calls
   *        turretIO.setVoltage(+1.0)
   *      for 0.5–1.0 seconds, then 0 volts. Bind it to a button.
   *
   * 2.2  With everyone clear of the robot, tap the button ONCE:
   *      - Observe direction: turret should move slowly and predictably.
   *      - Check that absoluteAngleDeg increases or decreases as expected.
   *      - Confirm that releasing the button stops motion immediately.
   *
   * 2.3  Repeat with
   *        turretIO.setVoltage(-1.0)
   *      to verify the opposite direction.
   *
   * 2.4  If motion is backwards relative to your coordinate system, FIX this
   *      here (invert motor or update angle sign convention) before trying
   *      closed‑loop.
   *
   * ---------------------------------------------------------------
   * PHASE 3 – CHECK SOFT LIMITS
   * ---------------------------------------------------------------
   * 3.1  Slowly drive the turret with small open‑loop voltages (±1–2 V)
   *      until you approach each mechanical hard‑stop.
   *
   * 3.2  Confirm:
   *      - You hit soft‑limit before the mechanical hard‑stop.
   *      - When at a soft‑limit, commanded motion further into that limit does
   *        NOT move (SparkMax holds position).
   *      - Coming back away from the limit works normally.
   *
   * 3.3  If soft‑limits are too tight or too loose, adjust hardware limits in
   *      this file AND turret coordinate mapping before continuing.
   *
   * ---------------------------------------------------------------
   * PHASE 4 – CLOSED‑LOOP ANGLE COMMANDS
   * ---------------------------------------------------------------
   * 4.1  With CRT verified and soft‑limits validated, test closed‑loop using
   *      small, known targets:
   *        turretIO.setAngle(0.0);
   *        turretIO.setAngle(+30.0);
   *        turretIO.setAngle(-30.0);
   *
   * 4.2  For each target:
   *      - Start from a nearby angle (do NOT jump across the entire range).
   *      - Verify that absoluteAngleDeg converges smoothly to the target
   *        without oscillation or overshoot into the hard‑stops.
   *      - If it oscillates, reduce kP or constrain the motion profile
   *        (m_constraints) BEFORE trying more aggressive commands.
   *
   * 4.3  Only after all of the above is stable should you:
   *      - Enable automatic aiming / shooter integration.
   *      - Allow field‑relative commands to control the turret.
   *
   * ---------------------------------------------------------------
   * QUICK SAFETY RESET
   * ---------------------------------------------------------------
   * At ANY sign of unsafe behavior (wrong direction, large jumps, buzzing):
   *   - Hit DISABLE on the Driver Station immediately.
   *   - Check encoder values and CRT error again.
   *   - Re‑run from PHASE 1 if you change wiring, offsets, or gearing.
   */

  public TurretIOReal(SparkMax auxSparkMax) {
    motor = new SparkMax(12, MotorType.kBrushless);
    auxSpark=auxSparkMax;
    encoder19 = motor.getAbsoluteEncoder();
    encoder21 = auxSpark.getAbsoluteEncoder();
    internalEncoder = motor.getEncoder();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    SparkMaxConfig auxConfig = new SparkMaxConfig();

    // Configure Main Motor
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false);

    // Conversion Factors
    double totalGearRatio = k_gearboxRatio * (k_turretRingTeeth / k_gear19);
    double positionFactor = 360.0 / totalGearRatio;
    motorConfig.encoder
        .positionConversionFactor(positionFactor)
        .velocityConversionFactor(positionFactor / 60.0);

    motorConfig.absoluteEncoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Hardware Soft Limits
    motorConfig.softLimit
        .forwardSoftLimit(360.0)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-360.0)
        .reverseSoftLimitEnabled(true);

    // Configure Aux Spark (Hood motor) - absolute encoder on data port for CRT
    auxConfig.absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    tryUntilOk(motor, 5, () -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(auxSpark, 5, () -> auxSpark.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    m_pidController.setTolerance(1.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    ifOk(motor, encoder19::getPosition, (val) -> inputs.absoluteEncoder19Pos = val);
    ifOk(auxSpark, encoder21::getPosition, (val) -> inputs.absoluteEncoder21Pos = val);
    
    // CRT Calculation
    double[] crtResult = calculateCrtAngle(inputs.absoluteEncoder19Pos, inputs.absoluteEncoder21Pos);
    inputs.absoluteAngleDeg = crtResult[0];
    inputs.crtError = crtResult[1];
    lastAbsoluteAngleDeg = inputs.absoluteAngleDeg;

    ifOk(motor, internalEncoder::getPosition, (val) -> inputs.motorPositionDeg = val);
    ifOk(motor, internalEncoder::getVelocity, (val) -> inputs.motorVelocityDegPerSec = val);
    
    ifOk(motor, new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (val) -> inputs.currentAmps = val);
    ifOk(motor, motor::getMotorTemperature, (val) -> inputs.temperatureCelsius = val);

    // Run Profiled PID calculation if in closed loop mode
    if (isClosedLoop) {
      double output = m_pidController.calculate(inputs.motorPositionDeg, targetAngleDegrees);
      motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }
  }

  private double[] calculateCrtAngle(double raw19, double raw21) {
    double r19 = ((raw19 - k_enc19Offset) % 1.0 + 1.0) % 1.0;
    double r21 = ((raw21 - k_enc21Offset) % 1.0 + 1.0) % 1.0;

    double bestError = Double.MAX_VALUE;
    double bestTurretDegrees = 0.0;

    for (int k = 0; k < 21; k++) {
      double totalRotations19 = k + r19;
      double turretRotations = totalRotations19 / (k_turretRingTeeth / k_gear19);
      double totalRotations21 = turretRotations * (k_turretRingTeeth / k_gear21);
      double expectedR21 = ((totalRotations21 % 1.0) + 1.0) % 1.0;

      double error = Math.abs(r21 - expectedR21);
      if (error > 0.5) error = 1.0 - error;

      if (error < bestError) {
        bestError = error;
        bestTurretDegrees = turretRotations * 360.0;
      }
    }

    double maxUniqueDeg = ((k_gear19 * k_gear21) / k_turretRingTeeth) * 360.0;
    if (bestTurretDegrees > (maxUniqueDeg / 2.0)) bestTurretDegrees -= maxUniqueDeg;

    return new double[] {bestTurretDegrees, bestError};
  }

  @Override
  public void setAngle(double degrees) {
    if (!isClosedLoop) {
      // Sync internal encoder to absolute position before starting closed loop
      internalEncoder.setPosition(lastAbsoluteAngleDeg);
      m_pidController.reset(lastAbsoluteAngleDeg);
      isClosedLoop = true;
    }
    targetAngleDegrees = degrees;
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    motor.setVoltage(volts);
  }

  @Override
  public void setInternalPosition(double degrees) {
    internalEncoder.setPosition(degrees);
    m_pidController.reset(degrees);
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    motor.stopMotor();
  }
}