# Subsystems

This directory contains the robot's subsystem implementations. Each subsystem is responsible for a specific part of the robot's functionality.

## Subsystem Structure

### Drive
- **Purpose**: Controls the robot's drivetrain
- **Components**:
  - Motor controllers
  - Gyro/IMU
  - Odometry
- **Key Features**:
  - Field-relative driving
  - Path following
  - Pose estimation

### Vision
- **Purpose**: Handles vision processing and target tracking
- **Components**:
  - Limelight 4 cameras
  - MegaTag 2 localization
- **Key Features**:
  - Multi-camera support
  - 3D pose estimation
  - Target filtering and validation

### Intake
- **Purpose**: Controls the game piece intake mechanism
- **Components**:
  - Intake motors
  - Sensors for game piece detection
- **Key Features**:
  - Automatic game piece detection
  - Speed control

### Shooter
- **Purpose**: Controls the shooting mechanism
- **Components**:
  - Shooter motors
  - Hood mechanism
  - Turret (if applicable)
- **Key Features**:
  - Variable shooting speeds
  - Auto-aiming
  - Distance-based power adjustment

### Hopper
- **Purpose**: Manages the transfer of game pieces from intake to shooter
- **Components**:
  - Conveyor belts
  - Indexing mechanism
  - Sensors for piece tracking

### Climb
- **Purpose**: Controls the climbing mechanism
- **Components**:
  - Climb motors
  - Pneumatics (if applicable)
  - Limit switches

### LED
- **Purpose**: Controls the robot's LED indicators
- **Components**:
  - LED strips
  - LED controllers
- **Key Features**:
  - Status indication
  - Animation patterns
  - Game piece detection feedback

## Implementation Notes

- Each subsystem follows the **IO Layer** pattern for hardware abstraction
- Subsystems should be hardware-agnostic where possible
- All hardware access should be done through the IO layer
- Use the `@AutoLog` annotation for automatic logging of important values
- Implement proper safety features and limits
- Add simulation support for testing without hardware

## Best Practices

1. **Encapsulation**: Keep hardware details hidden behind interfaces
2. **Thread Safety**: Ensure thread-safe access to hardware resources
3. **Error Handling**: Implement proper error handling and recovery
4. **Documentation**: Document all public methods and important implementation details
5. **Testing**: Include unit tests for all non-trivial functionality
