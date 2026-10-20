package frc.robot.subsystems.kicker;

/**
 * Simulation implementation of kickerIO. It does the same job as kickerIOReal, but instead of
 * talking to a motor controller it runs a physics model, so the code can be tested on a laptop.
 */
public class kickerIOSim implements kickerIO {
  // TODO: add fields for the simulation.
  //   - A DCMotorSim (edu.wpi.first.wpilibj.simulation.DCMotorSim) to model the motor.
  //   - A double to remember the voltage the subsystem asked for.
  //   - A loop period constant of 0.02 seconds (the robot loop runs every 20 ms).

  public kickerIOSim() {
    // TODO: build the motor model.
    //   DCMotorSim takes a LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),
    //   kickerConstants.MOI_KG_M2, kickerConstants.GEAR_RATIO) and the same DCMotor.
  }

  @Override
  public void updateInputs(kickerIOInputs inputs) {
    // TODO: advance the simulation, then report it like the real IO would.
    //   1. Give the model the voltage we were asked for:  motorSim.setInputVoltage(...)
    //   2. Step it forward one loop:                     motorSim.update(0.02)
    //   3. Copy position, velocity, applied volts and current draw into `inputs`
    //      (the same fields kickerIOReal fills in).
  }

  @Override
  public void setPercentIO(double percent) {
    // TODO: convert percent (-1..1) to volts (multiply by 12), clamp it to +/-12 with
    //   MathUtil.clamp, and store it in the field for updateInputs() to use.
  }

  @Override
  public void stop() {
    // TODO: set the stored voltage back to 0.
  }
}
