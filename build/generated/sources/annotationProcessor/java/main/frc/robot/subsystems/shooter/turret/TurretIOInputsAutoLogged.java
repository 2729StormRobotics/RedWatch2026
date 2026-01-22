package frc.robot.subsystems.shooter.turret;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretIOInputsAutoLogged extends TurretIO.TurretIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AbsolutePositionRotations", absolutePositionRotations);
    table.put("MotorPositionRotations", motorPositionRotations);
    table.put("MotorVelocityRotationsPerSec", motorVelocityRotationsPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    absolutePositionRotations = table.get("AbsolutePositionRotations", absolutePositionRotations);
    motorPositionRotations = table.get("MotorPositionRotations", motorPositionRotations);
    motorVelocityRotationsPerSec = table.get("MotorVelocityRotationsPerSec", motorVelocityRotationsPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  public TurretIOInputsAutoLogged clone() {
    TurretIOInputsAutoLogged copy = new TurretIOInputsAutoLogged();
    copy.absolutePositionRotations = this.absolutePositionRotations;
    copy.motorPositionRotations = this.motorPositionRotations;
    copy.motorVelocityRotationsPerSec = this.motorVelocityRotationsPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    return copy;
  }
}
