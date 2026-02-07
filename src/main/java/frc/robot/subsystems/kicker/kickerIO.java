package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface kickerIO {
  @AutoLog
  public static class kickerIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(kickerIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  default void setVoltage(double volts) {}

  /** Stop the motor. */
  default void stop() {}
}