package frc.robot.subsystems.samplemotor;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SampleMotorIOInputsAutoLogged extends SampleMotorIO.SampleMotorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRotations", positionRotations);
    table.put("VelocityRotationsPerSec", velocityRotationsPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRotations = table.get("PositionRotations", positionRotations);
    velocityRotationsPerSec = table.get("VelocityRotationsPerSec", velocityRotationsPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public SampleMotorIOInputsAutoLogged clone() {
    SampleMotorIOInputsAutoLogged copy = new SampleMotorIOInputsAutoLogged();
    copy.positionRotations = this.positionRotations;
    copy.velocityRotationsPerSec = this.velocityRotationsPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps.clone();
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
