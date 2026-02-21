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
  private double angleSetpointRad = 0.0;
  
  // Local tracking variables to match HoodIOReal logic
  private double positionSetpointRotations = 0.0;
  private boolean controllerEnabled = false;

  public HoodIOSim() {
    armSim = new SingleJointedArmSim(
        DCMotor.getNeo550(1),
        GEAR_RATIO,
        MOI_KG_M2,
        ARM_LENGTH_M,
        MIN_ANGLE_RAD,
        MAX_ANGLE_RAD,
        false, // Simulate gravity (false per constants)
        MIN_ANGLE_RAD); // Starting angle

    // Initialize setpoints to current state
    double initialAngle = armSim.getAngleRads();
    this.angleSetpointRad = initialAngle;
    this.positionSetpointRotations = (initialAngle * GEAR_RATIO) / (2.0 * Math.PI);

    // Initial state: Motor stopped, voltage 0
    armSim.setInputVoltage(0.0);
    controllerEnabled = false; 
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // 1. Calculate control voltage (Simulating the SparkMax internal PID)
    if (controllerEnabled) {
      double currentAngleRad = armSim.getAngleRads();
      double velocityRadPerSec = armSim.getVelocityRadPerSec();

      // PID constants tuned for the simulation model
      double kP = 12.0;
      double kD = 0.1;

      double error = angleSetpointRad - currentAngleRad;
      appliedVolts = MathUtil.clamp(kP * error - kD * velocityRadPerSec, -12.0, 12.0);
    } else {
      // If stopped, apply 0 volts (Simulates Brake Mode via Back-EMF)
      appliedVolts = 0.0;
    }

    // 2. Update Physics
    armSim.setInputVoltage(appliedVolts);
    armSim.update(LOOP_PERIOD_SECS);

    // 3. Update IO Inputs
    // Convert Mechanism Radians -> Motor Rotations
    double angleRad = armSim.getAngleRads();
    inputs.motorPositionRotations = (angleRad * GEAR_RATIO) / (2.0 * Math.PI);
    inputs.motorVelocityRotationsPerSec = (armSim.getVelocityRadPerSec() * GEAR_RATIO) / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
    // Normalized [0,1] for angle MIN to MAX (for getHoodCurrentAngle)
    inputs.absolutePositionRotations = MathUtil.clamp(
        (angleRad - MIN_ANGLE_RAD) / (MAX_ANGLE_RAD - MIN_ANGLE_RAD), 0.0, 1.0);
  }

  @Override
  public void setAngle(double angleRadians) {
    controllerEnabled = true;
    this.angleSetpointRad = MathUtil.clamp(angleRadians, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
    this.positionSetpointRotations = (angleSetpointRad * GEAR_RATIO) / (2.0 * Math.PI);
  }

  @Override
  public void setPosition(double targetRotations) {
    // Re-enable control loop
    controllerEnabled = true;
    this.positionSetpointRotations = targetRotations;

    // Convert Motor Rotations -> Mechanism Radians
    // Formula: (Rotations / GearRatio) * 2PI
    // Note: Algebraically equivalent to (Rotations * 2PI) / GearRatio
    this.angleSetpointRad = (targetRotations * 2.0 * Math.PI) / GEAR_RATIO;

    // Clamp to physical hard stops to prevent sim instability
    this.angleSetpointRad = MathUtil.clamp(angleSetpointRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
  }

  @Override
  public double getPosition() {
    // Convert Mechanism Radians -> Motor Rotations
    return (armSim.getAngleRads() * GEAR_RATIO) / (2.0 * Math.PI);
  }

  @Override
  public boolean isAtPosition(double target) {
    // Match tolerance from HoodIOReal (1.0 rotation)
    return Math.abs(getPosition() - target) < 1.0;
  }

  @Override
  public void stop() {
    // Disable controller (matches motor.stopMotor())
    controllerEnabled = false;
    appliedVolts = 0.0;
    
    // Update local setpoint variable to match "Real" behavior
    // (Real implementation updates this on stop for dashboard consistency)
    this.positionSetpointRotations = getPosition();
    
    armSim.setInputVoltage(0.0);
  }
}