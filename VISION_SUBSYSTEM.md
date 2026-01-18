# Vision Subsystem Documentation

## Overview

The Vision subsystem provides robust, hardware-agnostic vision processing for the 2026 robot using Limelight 4 cameras with MegaTag 2 localization.

## Architecture

The subsystem follows the **IO Layer pattern** for hardware abstraction:

- **VisionIO** - Interface defining vision hardware operations
- **VisionIOLimelight** - Real hardware implementation using Limelight cameras
- **VisionIOSim** - Simulation implementation (placeholder)
- **Vision** - Main subsystem that processes vision data and fuses with odometry

## Hardware Setup

### Cameras
- **Left Limelight**: NetworkTables name `"limelight-left"` (defined in `VisionConstants.LEFT_LIMELIGHT_NAME`)
- **Right Limelight**: NetworkTables name `"limelight-right"` (defined in `VisionConstants.RIGHT_LIMELIGHT_NAME`)

### MegaTag 2 Configuration
- Both cameras must be running the MegaTag 2 localization pipeline
- Robot orientation (gyro yaw and angular velocity) is automatically injected every cycle

## Key Features

### 1. MegaTag 2 Integration
- **Critical**: Robot orientation is updated every cycle via `setRobotOrientation()`
- This is handled automatically by the Vision subsystem
- Without this, MegaTag 2 will not function correctly

### 2. Dynamic Standard Deviations
The subsystem calculates trust levels based on vision quality:

- **Multiple Tags (≥2)**: High trust
  - Standard deviation: `MULTI_TAG_STD_DEV` (0.5m)
  
- **Single Tag, Close (< 4m)**: Medium trust
  - Standard deviation: Interpolated between `CLOSE_SINGLE_TAG_STD_DEV` (0.8m) and `FAR_SINGLE_TAG_STD_DEV` (2.0m)
  
- **Single Tag, Far (≥ 4m)**: Low trust
  - Standard deviation: `FAR_SINGLE_TAG_STD_DEV` (2.0m)

### 3. Rotation Trust
- **Rotation standard deviation**: Set to `INFINITY`
- Vision is **only used for X/Y translation**
- Gyro is trusted completely for rotation

### 4. Dual Camera Support
- Processes measurements from both left and right cameras
- Each camera's measurements are independently evaluated
- Both cameras contribute to pose estimation

## Usage

### Integration
The Vision subsystem is automatically initialized in `RobotContainer`:

```java
// Real robot
vision = new Vision(
    new VisionIOLimelight(VisionConstants.LEFT_LIMELIGHT_NAME),
    new VisionIOLimelight(VisionConstants.RIGHT_LIMELIGHT_NAME),
    drive);

// Simulation
vision = new Vision(
    new VisionIOSim(),
    new VisionIOSim(),
    drive);
```

### Automatic Operation
The Vision subsystem runs automatically in `periodic()`:
1. Gets gyro data from Drive subsystem
2. Updates robot orientation to both Limelight cameras
3. Reads pose estimates from cameras
4. Calculates dynamic standard deviations
5. Adds vision measurements to Drive's pose estimator

## Logging

All vision data is logged via AdvantageKit:

### Inputs (Auto-logged)
- `Vision/Left/*` - Left camera data
- `Vision/Right/*` - Right camera data
- Camera connection status
- Pose estimates
- Tag counts and distances
- Latency

### Outputs
- `Vision/GyroYawRad` - Current gyro yaw
- `Vision/GyroYawVelocityRadPerSec` - Current angular velocity
- `Vision/Left/StdDevX`, `StdDevY`, `StdDevTheta` - Calculated standard deviations for left camera
- `Vision/Right/StdDevX`, `StdDevY`, `StdDevTheta` - Calculated standard deviations for right camera
- `Vision/TotalTagCount` - Combined tag count from both cameras

## Configuration

Edit `VisionConstants.java` to adjust:

- Camera names
- Standard deviation values
- Maximum single tag distance threshold
- Minimum tags for high confidence

## LimelightHelpers Integration (Optional)

If you have the LimelightHelpers library, you can simplify `VisionIOLimelight.java`:

1. Replace `getBotPoseFromNetworkTables()` with:
   ```java
   LimelightHelpers.GetBotPose2d(limelightName)
   ```

2. Replace `setRobotOrientationToLimelight()` with:
   ```java
   LimelightHelpers.SetRobotOrientation(limelightName, yaw, yawVelocity)
   ```

The current implementation uses NetworkTables directly, which works without additional dependencies.

## Testing

### Simulation
- Vision subsystem runs but provides no vision data
- Can be used to test integration without hardware

### Replay
- Vision data from logs can be replayed
- Useful for debugging vision fusion logic

## Troubleshooting

### MegaTag 2 Not Working
- **Check**: Is `setRobotOrientation()` being called every cycle?
- **Solution**: Ensure Vision subsystem is running (it's automatic)

### Poor Pose Estimates
- **Check**: Tag count and distances in logs
- **Adjust**: Standard deviation constants in `VisionConstants.java`
- **Verify**: Camera pipeline is set to MegaTag 2

### No Vision Data
- **Check**: NetworkTables connection to Limelight
- **Verify**: Camera names match `VisionConstants`
- **Check**: Limelight is powered and on network

