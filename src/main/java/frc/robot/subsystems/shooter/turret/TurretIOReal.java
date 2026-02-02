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

  public TurretIOReal() {
    motor = new SparkMax(12, MotorType.kBrushless);
    auxSpark = new SparkMax(11, MotorType.kBrushless);

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

    // Configure Aux Spark
    auxConfig.absoluteEncoder
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
      m_pidController.reset(internalEncoder.getPosition());
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