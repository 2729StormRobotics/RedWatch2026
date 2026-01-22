package frc.robot.subsystems.hopper;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HopperIOInputsAutoLogged extends HopperIO.HopperIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
    table.put("BeamBreakTriggered", beamBreakTriggered);
  }

  @Override
  public void fromLog(LogTable table) {
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
    beamBreakTriggered = table.get("BeamBreakTriggered", beamBreakTriggered);
  }

  public HopperIOInputsAutoLogged clone() {
    HopperIOInputsAutoLogged copy = new HopperIOInputsAutoLogged();
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    copy.beamBreakTriggered = this.beamBreakTriggered;
    return copy;
  }
}
