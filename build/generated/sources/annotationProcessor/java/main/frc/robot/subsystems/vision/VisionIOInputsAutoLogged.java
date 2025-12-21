package frc.robot.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasPose", hasPose);
    table.put("PoseTimestamp", poseTimestamp);
    table.put("VisionPose", visionPose);
    table.put("TagCount", tagCount);
    table.put("AverageTagDistance", averageTagDistance);
    table.put("ClosestTagDistance", closestTagDistance);
    table.put("LatencyMs", latencyMs);
    table.put("MegaTag2Active", megaTag2Active);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasPose = table.get("HasPose", hasPose);
    poseTimestamp = table.get("PoseTimestamp", poseTimestamp);
    visionPose = table.get("VisionPose", visionPose);
    tagCount = table.get("TagCount", tagCount);
    averageTagDistance = table.get("AverageTagDistance", averageTagDistance);
    closestTagDistance = table.get("ClosestTagDistance", closestTagDistance);
    latencyMs = table.get("LatencyMs", latencyMs);
    megaTag2Active = table.get("MegaTag2Active", megaTag2Active);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.hasPose = this.hasPose;
    copy.poseTimestamp = this.poseTimestamp;
    copy.visionPose = this.visionPose;
    copy.tagCount = this.tagCount;
    copy.averageTagDistance = this.averageTagDistance;
    copy.closestTagDistance = this.closestTagDistance;
    copy.latencyMs = this.latencyMs;
    copy.megaTag2Active = this.megaTag2Active;
    return copy;
  }
}
