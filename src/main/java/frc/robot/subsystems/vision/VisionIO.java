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
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for vision subsystems using Limelight cameras with MegaTag 2.
 * This follows the AdvantageKit IO pattern for proper logging and simulation support.
 */
public interface VisionIO {
  /**
   * Auto-logged inputs for vision processing.
   * All fields in this class are automatically logged by AdvantageKit.
   */
  @AutoLog
  public static class VisionIOInputs {
    /** Whether the camera is connected and receiving data */
    public boolean connected = false;

    /** Whether a valid pose estimate is available */
    public boolean hasPose = false;

    /** Timestamp of the pose estimate in seconds (FPGA time) */
    public double poseTimestamp = 0.0;

    /** Robot pose estimate from vision */
    public Pose2d visionPose = new Pose2d();

    /** Number of tags detected in the current frame */
    public int tagCount = 0;

    /** Average distance to detected tags in meters */
    public double averageTagDistance = 0.0;

    /** Closest tag distance in meters */
    public double closestTagDistance = Double.MAX_VALUE;

    /** Latency of vision processing in milliseconds */
    public double latencyMs = 0.0;

    /** Whether MegaTag 2 is active */
    public boolean megaTag2Active = false;
  }

  /**
   * Updates the set of loggable inputs.
   * This should be called once per robot loop cycle.
   *
   * @param inputs The inputs object to update
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Sets the robot's orientation for MegaTag 2.
   * This MUST be called every cycle for MegaTag 2 to function correctly.
   *
   * @param yaw Yaw angle in radians
   * @param yawVelocity Angular velocity in radians per second
   */
  public default void setRobotOrientation(double yaw, double yawVelocity) {}

  /**
   * Configures the Limelight's IMU mode for MegaTag 2.
   * Mode 0: Use external IMU only (default)
   * Mode 1: Seed internal IMU with external IMU yaw
   * Mode 2: Use internal IMU for MT2 localization (recommended for best performance)
   *
   * @param mode IMU mode (0, 1, or 2)
   */
  public default void setIMUMode(int mode) {}

  /**
   * Gets the latest pose estimate from the camera.
   *
   * @return Pose estimate, or null if no valid estimate available
   */
  public default Pose2d getPose() {
    return null;
  }

  /**
   * Gets the number of tags currently detected.
   *
   * @return Number of tags detected
   */
  public default int getTagCount() {
    return 0;
  }

  /**
   * Gets the average distance to detected tags.
   *
   * @return Average distance in meters, or Double.MAX_VALUE if no tags
   */
  public default double getAverageTagDistance() {
    return Double.MAX_VALUE;
  }

  /**
   * Gets the closest tag distance.
   *
   * @return Closest tag distance in meters, or Double.MAX_VALUE if no tags
   */
  public default double getClosestTagDistance() {
    return Double.MAX_VALUE;
  }

  /**
   * Gets the latency of vision processing.
   *
   * @return Latency in milliseconds
   */
  public default double getLatencyMs() {
    return 0.0;
  }

  /**
   * Sets the throttle parameter for thermal management.
   * Lower values = higher framerate = more heat.
   * Higher values = lower framerate = less heat.
   *
   * @param throttle Throttle value (0 = full speed, 100-200 = throttled for disabled mode)
   */
  public default void setThrottle(int throttle) {}
}

