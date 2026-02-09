package frc.robot.subsystems.hopper;


import static frc.robot.subsystems.shooter.turret.TurretConstants.MOI_KG_M2;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Robust Simulation implementation of ClimbIO.
 * Updated to simulate a 2-motor system (Leader/Follower).
 */
public class HopperIOSim implements HopperIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  
  // Simulation uses a single plant representing the combined torque of both motors
  private final DCMotorSim motorSim;
  
  private double appliedVolts = 0.0;

  // Simulation hard stops (meters)
  private final double kMinExtension = 0.0;
  private final double kMaxExtension = 0.1; // Adjust based on your telescoping limit
  private final double kMetersPerRotation = 0.01; // Placeholder conversion

  public HopperIOSim() {
    // LinearSystemId for two NEO motors (representing leader and follower)
    // We use DCMotor.getNEO(2) to simulate the combined torque of the gearbox
    motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(2), MOI_KG_M2, 1.0),
        DCMotor.getNEO(2));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // 1. Calculate current position
    double currentPos = motorSim.getAngularPositionRotations() * kMetersPerRotation;
    
    // 2. Hard Stop Logic: Prevent driving past physical limits in simulation
    double effectiveVolts = appliedVolts;
    if (currentPos <= kMinExtension && appliedVolts < 0) {
      effectiveVolts = 0; // Can't retract past bottom
    } else if (currentPos >= kMaxExtension && appliedVolts > 0) {
      effectiveVolts = 0; // Can't extend past top
    }

    // 3. Update the simulation
    motorSim.setInputVoltage(effectiveVolts);
    motorSim.update(LOOP_PERIOD_SECS);

    // 4. Update Logged Inputs
    inputs.appliedVolts = effectiveVolts; 
    // Total current draw is the sum of both motors in the simulation
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
  }

  @Override
  public void setVoltage(double volts) {
    // Correctly clamp and store intended voltage
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }



  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}