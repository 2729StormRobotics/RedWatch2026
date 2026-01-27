# Commands

This directory contains command implementations for the 2026 robot. Commands are used to perform specific actions or sequences of actions on the robot.

## Available Commands

### AutoScore
- **Purpose**: Automatically scores a game piece based on the current field position and target location
- **Usage**: `new AutoScore(arm, elevator, intake, wrist, targetPose)`
- **Dependencies**: Arm, Elevator, Intake, and Wrist subsystems

### DriveCommands
- **Purpose**: Contains various drive-related commands for teleoperated and autonomous control
- **Includes**:
  - `drive()` - Basic teleop driving with field-relative control
  - `driveWithJoysticks()` - Joystick-based driving
  - `driveToPose()` - Drives to a specified pose on the field
  - `pathfindToPose()` - Uses pathfinding to navigate to a pose
  - `pathfindToPoseThenFollowPath()` - Combines pathfinding with path following

### EmergencyEject
- **Purpose**: Emergency command to eject game pieces
- **Usage**: `new EmergencyEject(intake, ejectSpeed)`
- **Dependencies**: Intake subsystem

### FeedForwardCharacterization
- **Purpose**: Performs feedforward characterization for drivetrain or other mechanisms
- **Usage**: `new FeedForwardCharacterization(drive, test, addData, onEnd)`
- **Dependencies**: Requires a `SimpleMotorFeedforward` and a test interface

### IntelligentCollection
- **Purpose**: Implements intelligent game piece collection with sensor feedback
- **Usage**: `new IntelligentCollection(intake, sensors)`
- **Dependencies**: Intake subsystem and sensor interface

## Creating New Commands

When creating new commands, follow these guidelines:

1. Extend `CommandBase` or use the `Command` factory methods
2. Document the command's purpose, parameters, and dependencies
3. Implement proper initialization, execution, and cleanup
4. Add proper error handling and safety checks
5. Test thoroughly in simulation before deploying to the robot

## Best Practices

- Keep commands focused on a single responsibility
- Use command groups for complex sequences
- Implement proper interrupt handling
- Add timeout protection for safety
- Use the command scheduler effectively for parallel and sequential operations
