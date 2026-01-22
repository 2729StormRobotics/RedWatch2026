package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotPositionRotations", pivotPositionRotations);
    table.put("PivotVelocityRotationsPerSec", pivotVelocityRotationsPerSec);
    table.put("PivotAppliedVolts", pivotAppliedVolts);
    table.put("PivotCurrentAmps", pivotCurrentAmps);
    table.put("RollerAppliedVolts", rollerAppliedVolts);
    table.put("RollerCurrentAmps", rollerCurrentAmps);
    table.put("BeamBreakTriggered", beamBreakTriggered);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotPositionRotations = table.get("PivotPositionRotations", pivotPositionRotations);
    pivotVelocityRotationsPerSec = table.get("PivotVelocityRotationsPerSec", pivotVelocityRotationsPerSec);
    pivotAppliedVolts = table.get("PivotAppliedVolts", pivotAppliedVolts);
    pivotCurrentAmps = table.get("PivotCurrentAmps", pivotCurrentAmps);
    rollerAppliedVolts = table.get("RollerAppliedVolts", rollerAppliedVolts);
    rollerCurrentAmps = table.get("RollerCurrentAmps", rollerCurrentAmps);
    beamBreakTriggered = table.get("BeamBreakTriggered", beamBreakTriggered);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.pivotPositionRotations = this.pivotPositionRotations;
    copy.pivotVelocityRotationsPerSec = this.pivotVelocityRotationsPerSec;
    copy.pivotAppliedVolts = this.pivotAppliedVolts;
    copy.pivotCurrentAmps = this.pivotCurrentAmps;
    copy.rollerAppliedVolts = this.rollerAppliedVolts;
    copy.rollerCurrentAmps = this.rollerCurrentAmps;
    copy.beamBreakTriggered = this.beamBreakTriggered;
    return copy;
  }
}
