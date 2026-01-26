package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Robust Simulation implementation of ClimbIO.
 * Fixed to ensure negative voltage is applied and hard stops are respected.
 */
public class ClimbIOSim implements ClimbIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final DCMotorSim motorSim;
  
  private double appliedVolts = 0.0;
  private boolean lockEngaged = false;

  // Simulation hard stops (meters)
  private final double kMinExtension = 0.0;
  private final double kMaxExtension = 0.1; // Adjust based on your telescoping limit
  private final double kMetersPerRotation = 0.01; // Placeholder conversion

  public ClimbIOSim() {
    // LinearSystemId for a single NEO motor
    motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), MOI_KG_M2, 1.0),
        DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // 1. Calculate current position
    double currentPos = motorSim.getAngularPositionRotations() * kMetersPerRotation;
    
    // 2. Hard Stop Logic: Prevent driving past physical limits in simulation
    double effectiveVolts = appliedVolts;
    if (currentPos <= kMinExtension && appliedVolts <= 0) {
      effectiveVolts = 10; // Can't retract past bottom
    } else if (currentPos >= kMaxExtension && appliedVolts >= 0) {
      effectiveVolts = -10; // Can't extend past top
    }
    // 3. Update the simulation
    motorSim.setInputVoltage(effectiveVolts);
    motorSim.update(LOOP_PERIOD_SECS);


    // 4. Update Logged Inputs
    inputs.appliedVolts = effectiveVolts; 
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
    inputs.lockEngaged = lockEngaged;
    inputs.positionMeters = currentPos;
  }

  @Override
  public void setVoltage(double volts) {
    // Correctly clamp and store intended voltage
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setLock(boolean engaged) {
    lockEngaged = engaged;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}