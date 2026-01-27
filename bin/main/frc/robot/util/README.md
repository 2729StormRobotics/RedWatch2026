# Utility Classes

This directory contains utility classes and helper functions used throughout the robot codebase.

## Core Utilities

### LocalADStarAK
- **Purpose**: Implements the AD* (Anytime Dynamic A*) pathfinding algorithm
- **Features**:
  - Dynamic path replanning
  - Efficient graph search
  - Integration with field coordinates
- **Usage**:
  ```java
  // Create a new pathfinder instance
  LocalADStarAK pathfinder = new LocalADStarAK(
      fieldWidth, fieldHeight, 
      nodeSizeMeters, 
      robotWidthMeters, 
      use8Way);
  
  // Update obstacles and find path
  pathfinder.updateObstacles(obstacles);
  List<Pose2d> path = pathfinder.getPath(startPose, endPose);
  ```

### PowerOrchestration
- **Purpose**: Manages power distribution and prioritization
- **Features**:
  - Dynamic power limiting
  - Priority-based power allocation
  - Real-time monitoring
- **Usage**:
  ```java
  // Create power manager
  PowerOrchestration powerManager = new PowerOrchestration(pdh);
  
  // Register components with priorities
  powerManager.registerComponent("drivetrain", 0.7, 1);  // High priority
  powerManager.registerComponent("intake", 0.3, 3);      // Lower priority
  
  // Update and manage power
  powerManager.update();
  ```

### SparkUtil
- **Purpose**: Utility functions for working with REV Spark MAX motor controllers
- **Features**:
  - Motor configuration helpers
  - Fault checking
  - Common setup patterns
- **Usage**:
  ```java
  // Configure a Spark MAX with common settings
  CANSparkMax motor = SparkUtil.createSparkMax(
      canId, 
      MotorType.kBrushless, 
      idleMode, 
      currentLimit, 
      invert
  );
  ```

## Subdirectories

### autonomous/
Contains utilities for autonomous routines and path following.

### drive/
Drive-specific utilities including odometry and kinematics helpers.

### misc/
Miscellaneous utilities that don't fit into other categories.

## Best Practices

1. **Reusability**: Keep utilities generic and reusable across the codebase
2. **Documentation**: Document all public methods and classes
3. **Testing**: Include unit tests for all utility functions
4. **Thread Safety**: Ensure thread safety for utilities that may be accessed from multiple threads
5. **Error Handling**: Implement proper error handling and validation

## Adding New Utilities

When adding a new utility:
1. Place it in the most appropriate subdirectory
2. Follow the existing code style
3. Add comprehensive documentation
4. Include usage examples
5. Add unit tests
