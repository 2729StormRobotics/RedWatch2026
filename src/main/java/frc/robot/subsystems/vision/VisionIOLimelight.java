// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Real hardware implementation of VisionIO using Limelight cameras with MegaTag 2.
 * 
 * <p>This implementation uses NetworkTables directly to communicate with Limelight cameras.
 * If you have LimelightHelpers library available, you can replace the NetworkTables calls
 * with LimelightHelpers methods for cleaner code:
 * <ul>
 *   <li>LimelightHelpers.GetBotPose2d(limelightName) instead of getBotPoseFromNetworkTables()</li>
 *   <li>LimelightHelpers.SetRobotOrientation(limelightName, yaw, yawVelocity) instead of setRobotOrientationToLimelight()</li>
 * </ul>
 *
 * <p>IMPORTANT: This implementation MUST call setRobotOrientation() every cycle
 * for MegaTag 2 to function correctly. The Vision subsystem handles this automatically.
 */
public class VisionIOLimelight implements VisionIO {
  private final String limelightName;
  private Pose2d lastPose = null;
  private double lastAverageDistance = Double.MAX_VALUE;

  /**
   * Creates a new VisionIOLimelight for a specific Limelight camera.
   *
   * @param limelightName The NetworkTables name of the Limelight (e.g., "limelight-front")
   */
  public VisionIOLimelight(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Check if Limelight is connected (tv = 0 means no valid target, but camera might still be connected)
    // We'll consider it connected if we can read data from it
    inputs.connected = true; // Limelight is always "connected" if it's on the network

    try {
      // Get pose estimate from MegaTag 2
      // LimelightHelpers.GetBotPose2d() returns the robot pose estimate
      // Note: This requires LimelightHelpers library
      Pose2d botPose = getBotPose2d();

      if (botPose != null && isValidPose(botPose)) {
        inputs.hasPose = true;
        inputs.visionPose = botPose;
        
        // Get tag count and latency from MegaTag 2 pose array
        // Format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
        int tagCount = getTagCountFromMegaTag2();
        double latencyMs = getLatencyMsFromMegaTag2();
        
        inputs.poseTimestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
        inputs.tagCount = tagCount;
        inputs.averageTagDistance = getAverageTagDistanceFromMegaTag2();
        inputs.closestTagDistance = inputs.averageTagDistance; // Use average as closest for now
        inputs.latencyMs = latencyMs;
        inputs.megaTag2Active = true;

        // Update cached values
        lastPose = botPose;
        lastAverageDistance = inputs.averageTagDistance;
      } else {
        // No valid pose
        inputs.hasPose = false;
        inputs.tagCount = 0;
        inputs.averageTagDistance = Double.MAX_VALUE;
        inputs.closestTagDistance = Double.MAX_VALUE;
        inputs.latencyMs = 0.0;
        inputs.megaTag2Active = false;
      }
    } catch (Exception e) {
      // Error reading from Limelight
      Logger.recordOutput("Vision/" + limelightName + "/Error", e.getMessage());
      inputs.connected = false;
      inputs.hasPose = false;
    }
  }

  @Override
  public void setRobotOrientation(double yaw, double yawVelocity) {
    // CRITICAL: Must call this every cycle for MegaTag 2 to work correctly
    // LimelightHelpers.SetRobotOrientation() injects gyro data into Limelight
    try {
      setRobotOrientationToLimelight(yaw, yawVelocity);
    } catch (Exception e) {
      Logger.recordOutput("Vision/" + limelightName + "/OrientationError", e.getMessage());
    }
  }

  @Override
  public Pose2d getPose() {
    return lastPose;
  }

  @Override
  public int getTagCount() {
    return getTagCountFromMegaTag2();
  }

  @Override
  public double getAverageTagDistance() {
    return lastAverageDistance;
  }

  @Override
  public double getClosestTagDistance() {
    // For now, use average distance as closest (can be improved by reading from MT2 array)
    return lastAverageDistance;
  }

  @Override
  public double getLatencyMs() {
    return getLatencyMsFromMegaTag2();
  }

  /**
   * Gets the bot pose from Limelight using LimelightHelpers.
   * This method wraps the LimelightHelpers call to handle potential library issues.
   *
   * @return Bot pose estimate, or null if not available
   */
  private Pose2d getBotPose2d() {
    try {
      // LimelightHelpers.GetBotPose2d(limelightName) returns Pose2d
      // If LimelightHelpers is not available, this will need to be implemented
      // using NetworkTables directly
      return getBotPoseFromNetworkTables();
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Gets bot pose from NetworkTables using MegaTag 2.
   * This reads directly from NetworkTables using the MegaTag 2-specific entry.
   * MegaTag 2 provides a pose estimate with timestamp and tag count information.
   */
  private Pose2d getBotPoseFromNetworkTables() {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // MegaTag 2 provides botpose_wpiblue_megatag2 (with MT2 suffix)
      // Format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
      double[] botPose = table.getEntry("botpose_wpiblue_megatag2").getDoubleArray(new double[11]);
      
      if (botPose.length >= 6) {
        // Format: [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
        double x = botPose[0];
        double y = botPose[1];
        double yaw = Math.toRadians(botPose[5]);
        
        // Validate pose is reasonable (field bounds check)
        if (x >= -1.0 && x <= 18.0 && y >= -1.0 && y <= 9.0) {
          return new Pose2d(x, y, Rotation2d.fromRadians(yaw));
        }
      }
    } catch (Exception e) {
      Logger.recordOutput("Vision/" + limelightName + "/NetworkTablesError", e.getMessage());
    }
    return null;
  }

  /**
   * Sets robot orientation to Limelight for MegaTag 2.
   * This is critical for MegaTag 2 functionality.
   * 
   * According to Limelight docs, SetRobotOrientation takes:
   * (limelightName, yawDeg, rollDeg, pitchDeg, yawVelDeg, rollVelDeg, pitchVelDeg)
   * We send yaw and yawVelocity, leaving roll/pitch at 0.
   */
  private void setRobotOrientationToLimelight(double yaw, double yawVelocity) {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // Set robot orientation for MegaTag 2
      // Format: [yawDeg, rollDeg, pitchDeg, yawVelDeg, rollVelDeg, pitchVelDeg]
      table.getEntry("robot_orientation_set").setDoubleArray(new double[] {
          Math.toDegrees(yaw), 0.0, 0.0,  // yaw, roll, pitch
          Math.toDegrees(yawVelocity), 0.0, 0.0  // yawVel, rollVel, pitchVel
      });
    } catch (Exception e) {
      Logger.recordOutput("Vision/" + limelightName + "/SetOrientationError", e.getMessage());
    }
  }

  /**
   * Gets the number of tags detected from MegaTag 2.
   * Reads from the botpose array which includes tag count.
   */
  private int getTagCountFromMegaTag2() {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // MegaTag 2 botpose array: [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
      double[] botPose = table.getEntry("botpose_wpiblue_megatag2").getDoubleArray(new double[11]);
      
      if (botPose.length >= 8) {
        // tagCount is at index 7
        return (int) botPose[7];
      }
      return 0;
    } catch (Exception e) {
      return 0;
    }
  }

  /**
   * Gets latency from MegaTag 2 pose array.
   */
  private double getLatencyMsFromMegaTag2() {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // MegaTag 2 botpose array: [x, y, z, roll, pitch, yaw, latency, tagCount, ...]
      double[] botPose = table.getEntry("botpose_wpiblue_megatag2").getDoubleArray(new double[11]);
      
      if (botPose.length >= 7) {
        // latency is at index 6 (in milliseconds)
        return botPose[6];
      }
      return 0.0;
    } catch (Exception e) {
      return 0.0;
    }
  }

  /**
   * Gets average tag distance from MegaTag 2 pose array.
   * Format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
   */
  private double getAverageTagDistanceFromMegaTag2() {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // MegaTag 2 botpose array: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
      double[] botPose = table.getEntry("botpose_wpiblue_megatag2").getDoubleArray(new double[11]);
      
      if (botPose.length >= 10) {
        // avgTagDist is at index 9 (in meters)
        return botPose[9];
      }
      return Double.MAX_VALUE;
    } catch (Exception e) {
      return Double.MAX_VALUE;
    }
  }


  /**
   * Validates that a pose estimate is reasonable.
   *
   * @param pose The pose to validate
   * @return True if the pose is valid
   */
  private boolean isValidPose(Pose2d pose) {
    // Check if pose is within reasonable field bounds
    // Field is approximately 16.5m x 8m, but allow some margin
    double x = pose.getX();
    double y = pose.getY();
    
    return x >= -1.0 && x <= 18.0 && y >= -1.0 && y <= 9.0;
  }

  @Override
  public void setThrottle(int throttle) {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // Set throttle parameter for thermal management
      // 0 = full speed (enabled mode)
      // 100-200 = throttled (disabled mode to reduce heat)
      // Limelight 4 uses the "throttle_set" NetworkTables entry
      table.getEntry("throttle_set").setNumber(throttle);
      
      Logger.recordOutput("Vision/" + limelightName + "/Throttle", throttle);
    } catch (Exception e) {
      Logger.recordOutput("Vision/" + limelightName + "/ThrottleError", e.getMessage());
    }
  }

  @Override
  public void setIMUMode(int mode) {
    try {
      var table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
          .getTable(limelightName);
      
      // Set IMU mode for MegaTag 2
      // Mode 0: Use external IMU only (default)
      // Mode 1: Seed internal IMU with external IMU yaw
      // Mode 2: Use internal IMU for MT2 localization (recommended)
      table.getEntry("imu_mode_set").setNumber(mode);
      
      Logger.recordOutput("Vision/" + limelightName + "/IMUMode", mode);
    } catch (Exception e) {
      Logger.recordOutput("Vision/" + limelightName + "/IMUModeError", e.getMessage());
    }
  }
}

