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

/**
 * Simulation implementation of VisionIO.
 * Acts as a placeholder that provides no vision data.
 * In simulation, vision can be simulated using WPILib's PhotonVision simulation
 * or by manually injecting pose estimates for testing.
 */
public class VisionIOSim implements VisionIO {
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Simulation: No vision data available
    inputs.connected = false;
    inputs.hasPose = false;
    inputs.visionPose = new Pose2d();
    inputs.poseTimestamp = 0.0;
    inputs.tagCount = 0;
    inputs.averageTagDistance = Double.MAX_VALUE;
    inputs.closestTagDistance = Double.MAX_VALUE;
    inputs.latencyMs = 0.0;
    inputs.megaTag2Active = false;
  }

  @Override
  public void setRobotOrientation(double yaw, double yawVelocity) {
    // No-op in simulation
  }

  @Override
  public Pose2d getPose() {
    return null;
  }

  @Override
  public int getTagCount() {
    return 0;
  }

  @Override
  public double getAverageTagDistance() {
    return Double.MAX_VALUE;
  }

  @Override
  public double getClosestTagDistance() {
    return Double.MAX_VALUE;
  }

  @Override
  public double getLatencyMs() {
    return 0.0;
  }

  @Override
  public void setThrottle(int throttle) {
    // No-op in simulation
  }

  @Override
  public void setIMUMode(int mode) {
    // No-op in simulation
  }
}

