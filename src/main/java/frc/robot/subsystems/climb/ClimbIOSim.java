package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;
  private boolean lockEngaged = false;

  public ClimbIOSim() {
    motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), MOI_KG_M2, 1.0),
        DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    motorSim.update(LOOP_PERIOD_SECS);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
    inputs.lockEngaged = lockEngaged;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setLock(boolean engaged) {
    lockEngaged = engaged;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    motorSim.setInputVoltage(0.0);
  }
}
