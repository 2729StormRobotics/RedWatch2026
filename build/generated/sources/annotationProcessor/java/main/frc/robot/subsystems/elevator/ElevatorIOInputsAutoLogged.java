package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("SetpointMeters", setpointMeters);
    table.put("PositionMeters", positionMeters);
    table.put("VelocityMetersPerSec", velocityMetersPerSec);
    table.put("AppliedVoltage", appliedVoltage);
    table.put("MotorTemperature", motorTemperature);
    table.put("MotorCurrent", motorCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    setpointMeters = table.get("SetpointMeters", setpointMeters);
    positionMeters = table.get("PositionMeters", positionMeters);
    velocityMetersPerSec = table.get("VelocityMetersPerSec", velocityMetersPerSec);
    appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
    motorTemperature = table.get("MotorTemperature", motorTemperature);
    motorCurrent = table.get("MotorCurrent", motorCurrent);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.setpointMeters = this.setpointMeters;
    copy.positionMeters = this.positionMeters;
    copy.velocityMetersPerSec = this.velocityMetersPerSec;
    copy.appliedVoltage = this.appliedVoltage;
    copy.motorTemperature = this.motorTemperature.clone();
    copy.motorCurrent = this.motorCurrent.clone();
    return copy;
  }
}
