package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean beamBreakTriggered = false;
  }

  public default void updateInputs(HopperIOInputs inputs) {}
  public default void setVoltage(double volts) {}
  public default void stop() {}
}
