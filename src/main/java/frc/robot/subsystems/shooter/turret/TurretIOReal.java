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
import frc.robot.Constants;
import frc.robot.util.SparkIdleModeTuner;
import frc.robot.util.misc.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

/**
 * Real hardware implementation of TurretIO.
 * This class contains the CRT logic for absolute positioning AND the 
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
      new ProfiledPIDController(kP, kI, kD, m_constraints);

  private double targetAngleDegrees = 0.0;

  private boolean isClosedLoop = false;
  private double lastAbsoluteAngleDeg = 0.0;
  /** True once we've used CRT once at startup to seed the internal (relative) encoder. */
  private boolean initializedFromCrt = false;

  // Tunable PID gains for turret RIO-side controller
  private final LoggedTunableNumber kP_tunable =
      new LoggedTunableNumber("Shooter/Turret/kP", kP);
  private final LoggedTunableNumber kI_tunable =
      new LoggedTunableNumber("Shooter/Turret/kI", kI);
  private final LoggedTunableNumber kD_tunable =
      new LoggedTunableNumber("Shooter/Turret/kD", kD);

  public TurretIOReal(SparkMax auxSparkMax) {
    motor = new SparkMax(12, MotorType.kBrushless);
    auxSpark=auxSparkMax;
    encoder19 = motor.getAbsoluteEncoder();
    encoder21 = auxSpark.getAbsoluteEncoder();
    internalEncoder = motor.getEncoder();

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure Main Motor
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(true);

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
        .forwardSoftLimit(180)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-90)
        .reverseSoftLimitEnabled(true);

    tryUntilOk(motor, 5, () -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // NOTE: We intentionally do NOT reconfigure auxSpark here. That Spark MAX is owned by the hood
    // subsystem, and resetting safe parameters here can wipe its closed-loop config or change how its
    // data-port absolute encoder is interpreted. We only read its absolute encoder for CRT.

    m_pidController.setTolerance(1.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    ifOk(motor, encoder19::getPosition, (val) -> inputs.absoluteEncoder19Pos = val);
    ifOk(auxSpark, encoder21::getPosition, (val) -> inputs.absoluteEncoder21Pos = val);

    // CRT gives an absolute turret angle in degrees in [-180, 180] plus a consistency error.
    double[] crtResult = calculateCrtAngle(inputs.absoluteEncoder19Pos, inputs.absoluteEncoder21Pos);
    double crtAngleDeg =
        MathUtil.inputModulus(crtResult[0] + TurretConstants.ZERO_OFFSET_DEG, -180.0, 180.0);
    inputs.crtError = crtResult[1];

    ifOk(
        motor,
        internalEncoder::getPosition,
        (val) -> {
          inputs.motorPositionDeg = val;
          inputs.absoluteAngleDeg = val;
          lastAbsoluteAngleDeg = val;
        });
    ifOk(motor, internalEncoder::getVelocity, (val) -> inputs.motorVelocityDegPerSec = val);

    // Seed the internal encoder ONCE from CRT, then use internal encoder for smooth continuous angle.
    // This avoids occasional CRT "branch" jumps due to ambiguity/noise.
    if (!initializedFromCrt) {
      internalEncoder.setPosition(crtAngleDeg);
      m_pidController.reset(crtAngleDeg);
      inputs.motorPositionDeg = crtAngleDeg;
      inputs.absoluteAngleDeg = crtAngleDeg;
      lastAbsoluteAngleDeg = crtAngleDeg;
      initializedFromCrt = true;
    } else {
      // If the internal encoder ever resets (Spark reboot/brownout), the reported angle will jump.
      // Detect *large* disagreement while idle and snap back to CRT.
      final double crtTrustThreshold = 0.05; // lower is better
      final double resyncThresholdDeg = 90.0; // only resync on "obvious reset"
      if (!isClosedLoop && inputs.crtError >= 0.0 && inputs.crtError < crtTrustThreshold) {
        double diffDeg =
            Math.abs(MathUtil.inputModulus(crtAngleDeg - inputs.motorPositionDeg, -180.0, 180.0));
        if (diffDeg > resyncThresholdDeg) {
          internalEncoder.setPosition(crtAngleDeg);
          m_pidController.reset(crtAngleDeg);
          inputs.motorPositionDeg = crtAngleDeg;
          inputs.absoluteAngleDeg = crtAngleDeg;
          lastAbsoluteAngleDeg = crtAngleDeg;
        }
      }
    }
    
    ifOk(motor, new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (val) -> inputs.currentAmps = val);
    ifOk(motor, motor::getMotorTemperature, (val) -> inputs.temperatureCelsius = val);

    // Allow runtime brake/coast selection for turret motor.
    SparkIdleModeTuner.syncIdleMode(motor, "Shooter/TurretBrake", IdleMode.kBrake);

    // Turret PID tuning from Elastic / SmartDashboard when in tuning mode.
    if (Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          this.hashCode(),
          values -> {
            double p = values[0];
            double i = values[1];
            double d = values[2];
            m_pidController.setPID(p, i, d);
          },
          kP_tunable, kI_tunable, kD_tunable);
    }

    // Run Profiled PID calculation if in closed loop mode
    if (isClosedLoop) {
      double output = m_pidController.calculate(inputs.motorPositionDeg, targetAngleDegrees);
      motor.set(MathUtil.clamp(output, -.8, 0.8));
    }
  }

  // -------------------------------------------------------------------------
  // CRT (Coarse Relative Tracking) math — exact derivation
  // -------------------------------------------------------------------------
  // Physical model: 200T ring drives 19T (encoder A) and 21T (encoder B).
  // One turret rotation => 19T rotates 200/19 turns, 21T rotates 200/21 turns.
  // So:  totalRotations19 = turretRotations * (200/19)
  //      totalRotations21 = turretRotations * (200/21)  =>  totalRotations21 = totalRotations19 * (19/21)
  // Absolute encoders report frac in [0,1). So totalRotations19 = k + r19 (unknown int k).
  // We search k; for each k we get turretRotations = (k+r19)/(200/19), then expectedR21 = frac((k+r19)*19/21).
  // Pick k that minimizes circular distance between r21 and expectedR21.
  // Tie-break: when errors are equal (fp noise), prefer angle closest to previous (continuity).
  // -------------------------------------------------------------------------

  /** Fractional part in [0, 1). x - floor(x) is unambiguous for positive and negative x. */
  private static double frac(double x) {
    double f = x - Math.floor(x);
    return (f >= 1.0 - 1e-12) ? 0.0 : f;
  }

  /** Circular distance on [0, 1): min(|a-b|, 1 - |a-b|). */
  private static double circularError(double a, double b) {
    double diff = Math.abs(a - b);
    return (diff > 0.5) ? (1.0 - diff) : diff;
  }

  private static final double CRT_TIE_EPSILON = 1e-9;

  private double[] calculateCrtAngle(double raw19, double raw21) {
    double r19 = frac(raw19 - k_enc19Offset);
    double r21 = frac(raw21 - k_enc21Offset);

    double bestError = Double.MAX_VALUE;
    double bestTurretDegrees = 0.0;

    // 200/19 = encoder 19 rotations per turret rotation; 200/21 = encoder 21 rotations per turret rotation
    final double inv19 = k_gear19 / k_turretRingTeeth;   // 19/200
    final double ratio19to21 = k_gear19 / k_gear21;       // 19/21

    for (int k = -15; k <= 15; k++) {
      double totalRotations19 = k + r19;
      double turretRotations = totalRotations19 * inv19;
      double totalRotations21 = totalRotations19 * ratio19to21;

      double expectedR21 = frac(totalRotations21);
      double error = circularError(r21, expectedR21);

      double candidateDegrees = -turretRotations * 360.0;
      boolean strictlyBetter = error < bestError;
      boolean tie = (error <= bestError + CRT_TIE_EPSILON);
      double candidateDistToLast =
          Math.abs(MathUtil.inputModulus(candidateDegrees - lastAbsoluteAngleDeg, -180.0, 180.0));
      double bestDistToLast =
          Math.abs(MathUtil.inputModulus(bestTurretDegrees - lastAbsoluteAngleDeg, -180.0, 180.0));
      boolean closerToLast = tie && (candidateDistToLast < bestDistToLast);

      if (strictlyBetter || closerToLast) {
        bestError = error;
        bestTurretDegrees = candidateDegrees;
      }
    }

    double finalAngle = MathUtil.inputModulus(bestTurretDegrees, -180.0, 180.0);
    return new double[] {finalAngle, bestError};
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

  @Override
  public double getDesiredAngle() {
    return targetAngleDegrees;
  }
}