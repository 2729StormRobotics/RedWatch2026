package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/**
 * The IO interface is the contract between the subsystem and the hardware. The subsystem only ever
 * calls these methods; kickerIOReal fills them in for the real robot, kickerIOSim fills them in
 * for the simulator, and an empty {@code new kickerIO() {}} is used for log replay.
 *
 * <p>Every method has an empty default body so an implementation only overrides what it needs.
 */
public interface kickerIO {
  /**
   * Everything we want to READ from the motor. @AutoLog makes AdvantageKit generate a class called
   * kickerIOInputsAutoLogged that logs every field here automatically. To log something new, just
   * add a public field.
   */
  @AutoLog
  public static class kickerIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Fill in the inputs object with the latest readings. Called once per loop by the subsystem. */
  default void updateInputs(kickerIOInputs inputs) {}

  /** Run the motor at the given percent output (-1.0 to 1.0). */
  default void setPercentIO(double percent) {}

  /** Stop the motor. */
  default void stop() {}
}
