package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean lockEngaged = false;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}
  public default void setVoltage(double volts) {}
  public default void setLock(boolean engaged) {}
  public default void stop() {}
}
