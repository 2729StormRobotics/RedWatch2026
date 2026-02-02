package frc.robot.subsystems.shooter.flywheel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FlywheelIOInputsAutoLogged extends FlywheelIO.FlywheelIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderPositionRotations", leaderPositionRotations);
    table.put("LeaderVelocityRotationsPerSec", leaderVelocityRotationsPerSec);
    table.put("LeaderAppliedVolts", leaderAppliedVolts);
    table.put("LeaderCurrentAmps", leaderCurrentAmps);
    table.put("FollowerPositionRotations", followerPositionRotations);
    table.put("FollowerVelocityRotationsPerSec", followerVelocityRotationsPerSec);
    table.put("FollowerAppliedVolts", followerAppliedVolts);
    table.put("FollowerCurrentAmps", followerCurrentAmps);
    table.put("LeaderTemperatureCelsius", leaderTemperatureCelsius);
    table.put("FollowerTemperatureCelsius", followerTemperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderPositionRotations = table.get("LeaderPositionRotations", leaderPositionRotations);
    leaderVelocityRotationsPerSec = table.get("LeaderVelocityRotationsPerSec", leaderVelocityRotationsPerSec);
    leaderAppliedVolts = table.get("LeaderAppliedVolts", leaderAppliedVolts);
    leaderCurrentAmps = table.get("LeaderCurrentAmps", leaderCurrentAmps);
    followerPositionRotations = table.get("FollowerPositionRotations", followerPositionRotations);
    followerVelocityRotationsPerSec = table.get("FollowerVelocityRotationsPerSec", followerVelocityRotationsPerSec);
    followerAppliedVolts = table.get("FollowerAppliedVolts", followerAppliedVolts);
    followerCurrentAmps = table.get("FollowerCurrentAmps", followerCurrentAmps);
    leaderTemperatureCelsius = table.get("LeaderTemperatureCelsius", leaderTemperatureCelsius);
    followerTemperatureCelsius = table.get("FollowerTemperatureCelsius", followerTemperatureCelsius);
  }

  public FlywheelIOInputsAutoLogged clone() {
    FlywheelIOInputsAutoLogged copy = new FlywheelIOInputsAutoLogged();
    copy.leaderPositionRotations = this.leaderPositionRotations;
    copy.leaderVelocityRotationsPerSec = this.leaderVelocityRotationsPerSec;
    copy.leaderAppliedVolts = this.leaderAppliedVolts;
    copy.leaderCurrentAmps = this.leaderCurrentAmps;
    copy.followerPositionRotations = this.followerPositionRotations;
    copy.followerVelocityRotationsPerSec = this.followerVelocityRotationsPerSec;
    copy.followerAppliedVolts = this.followerAppliedVolts;
    copy.followerCurrentAmps = this.followerCurrentAmps;
    copy.leaderTemperatureCelsius = this.leaderTemperatureCelsius;
    copy.followerTemperatureCelsius = this.followerTemperatureCelsius;
    return copy;
  }
}
