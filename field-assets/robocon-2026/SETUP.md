# Robocon 2026 setup

Source: https://github.com/owen1050/2026-Robocon-Apriltag-Field-Info
Reference commit: 0f30f376c4fe3b44dc09af2ff792fb8e9c6b9d4b

Robot code loads src/main/deploy/2026-robocon-welded-photonvision-wpilib.json,
uses its dimensions for alliance flipping and vision bounds, and publishes each
AprilTag as SmartDashboard/Tag<ID> and Field/AprilTags/Tag<ID> in AdvantageKit.
Deploy the project with WPILib after building it.

## Limelight (required for actual localization)

Upload Limelight/ROBOCON_WELDED_LIMELIGHT.fmap through the custom field-map
upload control in each camera's web interface: limelight-front and limelight-back.
Select/save the custom map and verify it is active on both cameras. The robot-side
JSON does not configure the cameras. Preserve each camera's calibrated robot-relative
position and orientation. Check the reported blue-origin pose at a measured field
location before driving autonomously.

## PathPlanner

In Settings > App Settings > Field Image > Import Custom, name the field
2026Robocon, select FieldImages/robocon2026FieldImage.png, and enter
47.7481404921 pixels/meter, following the upstream instructions.
Create a separate git branch before editing paths. The existing coral autonomous
paths and navgrid are from the old field: rebuild paths and the obstacle grid for
Robocon in PathPlanner before using autonomous/pathfinding. Robot dimensions,
motor configuration, and tuning remain specific to this robot.

## AdvantageScope

Choose App > Show Assets Folder and copy AdvantageScope/Field2d_2026Robocon
into the custom assets folder. Select the 2026 Robocon field in a 2D field tab
and add SmartDashboard/Tag<ID> entries to inspect tag placement. Add the robot's
Odometry/Odometry pose to view localization.

## Reference discrepancy

Both WPILib and Limelight maps specify 14.6812 m x 8.069 m; this implementation
uses those values as its source of truth. The upstream sample sets fieldSizeX to
15.68 m, and its navgrid still says 16.54 m. Those conflicting values were not
copied. Confirm physical field dimensions with the event before autonomous use.
The map has 32 tags: IDs 15/16/31/32 are replaced by 33/34/49/50.

## Validation

The imported JSON and Limelight map dimensions and tag IDs were checked locally.
Compilation requires a Java 17/WPILib development environment; this workspace's
shell has no Java runtime, so compilation and hardware verification remain pending.
