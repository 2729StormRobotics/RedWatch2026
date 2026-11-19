package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of kickerIO. It does the same job as kickerIOReal, but instead of
 * talking to a motor controller it runs a physics model, so the code can be tested on a laptop.
 */
public class kickerIOSim implements kickerIO {
   private static final double LOOP_PERIOD_SECS = 0.02; // robot loop runs every 20 ms

  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0; // voltage the subsystem last asked for

  public kickerIOSim() {
    DCMotor gearbox = DCMotor.getNeoVortex(1);
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, kickerConstants.MOI_KG_M2, kickerConstants.GEAR_RATIO),
            gearbox);
  }

  @Override
  public void updateInputs(kickerIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(LOOP_PERIOD_SECS);

    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.velocityRotationsPerSec = motorSim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setPercentIO(double percent) {
      appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
  }

 @Override
  public void setPercentIO(double percent) {
    appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
  }

  @Override
  public void stop() {
    appliedvolts = 0.0;
  }
}
