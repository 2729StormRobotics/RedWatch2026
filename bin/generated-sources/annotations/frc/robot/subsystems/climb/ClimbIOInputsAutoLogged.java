package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
    table.put("LockEngaged", lockEngaged);
  }

  @Override
  public void fromLog(LogTable table) {
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
    lockEngaged = table.get("LockEngaged", lockEngaged);
  }

  public ClimbIOInputsAutoLogged clone() {
    ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    copy.lockEngaged = this.lockEngaged;
    return copy;
  }
}
