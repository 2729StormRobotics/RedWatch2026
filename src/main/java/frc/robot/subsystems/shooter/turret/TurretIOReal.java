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
        .forwardSoftLimit(180)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-90)
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
    // Inside updateInputs
    if (isClosedLoop) {
      // Use the MOTOR's relative encoder for the PID loop. 
      // It is 100x smoother and has no backlash relative to the motor shaft.
      double output = m_pidController.calculate(inputs.motorPositionDeg, targetAngleDegrees);
      motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }
    
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
      motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }
  }

  private double[] calculateCrtAngle(double raw19, double raw21) {
    // Normalize encoder readings into [0, 1) range
    double r19 = ((raw19 - k_enc19Offset) % 1.0 + 1.0) % 1.0;
    double r21 = ((raw21 - k_enc21Offset) % 1.0 + 1.0) % 1.0;

    double bestError = Double.MAX_VALUE;
    double bestTurretDegrees = 0.0;

    // Search across possible wraps
    for (int k = -15; k <= 15; k++) {
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

    // NEW LOGIC: Force the angle into the -180 to 180 range.
    // This ensures that if the turret is 10 degrees to the right, it shows -10,
    // and if it's 10 degrees to the left, it shows 10 (or vice versa depending on inversion).
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
}