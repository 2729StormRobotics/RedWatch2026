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
    table.put("AbsoluteEncoder19Pos", absoluteEncoder19Pos);
    table.put("AbsoluteEncoder21Pos", absoluteEncoder21Pos);
    table.put("AbsoluteAngleDeg", absoluteAngleDeg);
    table.put("CrtError", crtError);
    table.put("MotorPositionDeg", motorPositionDeg);
    table.put("MotorVelocityDegPerSec", motorVelocityDegPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    absolutePositionRotations = table.get("AbsolutePositionRotations", absolutePositionRotations);
    motorPositionRotations = table.get("MotorPositionRotations", motorPositionRotations);
    motorVelocityRotationsPerSec = table.get("MotorVelocityRotationsPerSec", motorVelocityRotationsPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
    absoluteEncoder19Pos = table.get("AbsoluteEncoder19Pos", absoluteEncoder19Pos);
    absoluteEncoder21Pos = table.get("AbsoluteEncoder21Pos", absoluteEncoder21Pos);
    absoluteAngleDeg = table.get("AbsoluteAngleDeg", absoluteAngleDeg);
    crtError = table.get("CrtError", crtError);
    motorPositionDeg = table.get("MotorPositionDeg", motorPositionDeg);
    motorVelocityDegPerSec = table.get("MotorVelocityDegPerSec", motorVelocityDegPerSec);
  }

  public TurretIOInputsAutoLogged clone() {
    TurretIOInputsAutoLogged copy = new TurretIOInputsAutoLogged();
    copy.absolutePositionRotations = this.absolutePositionRotations;
    copy.motorPositionRotations = this.motorPositionRotations;
    copy.motorVelocityRotationsPerSec = this.motorVelocityRotationsPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    copy.temperatureCelsius = this.temperatureCelsius;
    copy.absoluteEncoder19Pos = this.absoluteEncoder19Pos;
    copy.absoluteEncoder21Pos = this.absoluteEncoder21Pos;
    copy.absoluteAngleDeg = this.absoluteAngleDeg;
    copy.crtError = this.crtError;
    copy.motorPositionDeg = this.motorPositionDeg;
    copy.motorVelocityDegPerSec = this.motorVelocityDegPerSec;
    return copy;
  }
}
