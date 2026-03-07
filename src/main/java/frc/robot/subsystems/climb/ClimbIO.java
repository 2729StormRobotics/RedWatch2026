package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean lockEngaged = false;
    /** Position in meters (0.0 = fully retracted, positive = extended) */
    public double positionMeters = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}
  public default void setVoltage(double volts) {}
  public default void setLock(boolean engaged) {}
  public default void stop() {}
  public default void setPercent(double percent) {}
}