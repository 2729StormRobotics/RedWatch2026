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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.useVision;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.kPathConstraints;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthX;
import static frc.robot.subsystems.drive.DriveConstants.kTrackWidthY;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import frc.robot.util.autonomous.DeadzoneChooser;
import frc.robot.util.autonomous.LocalADStarAK;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class Drive extends SubsystemBase {
  // private static final double DRIVE_BASE_RADIUS = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY /
  // 2.0);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  // private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutDistance m_distance = Meters.mutable(0);
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
  private static final double MAX_ANGULAR_SPEED = kMaxSpeedMetersPerSecond / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  // private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Pose2d lastPose = new Pose2d();
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          rawGyroRotation,
          // getPose().getRotation(),
          lastModulePositions);

  private final SysIdRoutine sysId =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                for (Module module : modules) {
                  module.runCharacterization(voltage.in(Volts));
                }
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-front-left")
                    .voltage(modules[0].getDriveVoltage())
                    .linearPosition(Meters.of(modules[0].getPositionMeters()))
                    .linearVelocity(MetersPerSecond.of(modules[0].getVelocityMetersPerSec()));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-front-right")
                    .voltage(modules[1].getDriveVoltage())
                    .linearPosition(Meters.of(modules[1].getPositionMeters()))
                    .linearVelocity(MetersPerSecond.of(modules[1].getVelocityMetersPerSec()));

                log.motor("drive-back-left")
                    .voltage(modules[2].getDriveVoltage())
                    .linearPosition(Meters.of(modules[2].getPositionMeters()))
                    .linearVelocity(MetersPerSecond.of(modules[2].getVelocityMetersPerSec()));

                log.motor("drive-back-right")
                    .voltage(modules[3].getDriveVoltage())
                    .linearPosition(Meters.of(modules[3].getPositionMeters()))
                    .linearVelocity(MetersPerSecond.of(modules[3].getVelocityMetersPerSec()));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state
              // in
              // WPILog with this subsystem's name ("drive")
              this));
  private SysIdRoutine turnRoutine;

  private Rotation2d simRotation = new Rotation2d();

  private DeadzoneChooser deadzoneChooser = new DeadzoneChooser("Deadzone");

  private LoggedDashboardBoolean useVisionDashboard = new LoggedDashboardBoolean("UseVision", true);

  RobotConfig robotConfig;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    SparkMaxOdometryThread.getInstance().start();


    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    

    Pathfinding.setPathfinder(new LocalADStarAK());
    Pathfinding.ensureInitialized();

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    turnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> {
                  for (Module module : modules) {
                    module.runCharacterization(volts.in(Volts));
                  }
                },
                null,
                this));

    useVisionDashboard.set(useVision);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        robotConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    if (DriverStation.isAutonomousEnabled()) {
      // Pathfinding.setDynamicObstacles(deadzoneChooser.getDeadzone(), getPose().getTranslation());
      Pathfinding.setDynamicObstacles(List.of(), null);

    } else {
      Pathfinding.setDynamicObstacles(List.of(), null);
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
    }

    Logger.recordOutput("FieldVelocity", getFieldVelocity());

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else if (Constants.getRobotMode() == Constants.Mode.SIM) {
      rawGyroRotation = simRotation;
    } else {
      // rawGyroRotation = simRotation;
    }

    // poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);
    odometry.update(rawGyroRotation, modulePositions);

    Logger.recordOutput("Odometry/Odometry", odometry.getPoseMeters());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    simRotation =
        simRotation.rotateBy(Rotation2d.fromRadians(discreteSpeeds.omegaRadiansPerSecond * 0.02));
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxSpeedMetersPerSecond);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public Command resetYaw() {
    gyroIO.zeroAll();
    return null;
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and driveZ velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getRotation());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  /* public Rotation2d getRotation() {
    return getPose().getRotation();
  } */
  public Rotation2d getRotation() {
    // return new Rotation2d(gyroIO.getYawAngle());
    return getPose().getRotation();
    // return new Rotation2d(); // use if nothing works
  }

  public void updateDeadzoneChooser() {
    deadzoneChooser.init();
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command turnQuasistatic(SysIdRoutine.Direction direction) {
    return turnRoutine.quasistatic(direction);
  }

  public Command turnDynamic(SysIdRoutine.Direction direction) {
    return turnRoutine.dynamic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxSpeedMetersPerSecond;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
    };
  }

  /* Configure trajectory following */
  public Command goToPose(Pose2d target_pose, double end_velocity, double time_before_turn) {
    return AutoBuilder.pathfindToPose(
        target_pose, kPathConstraints, end_velocity);
  }

  public Command goToPose(Pose2d target_pose) {
    return AutoBuilder.pathfindToPose(target_pose, kPathConstraints, 0.0);
  }

  public Command getAuto(String nameString) {
    return AutoBuilder.buildAuto(nameString);
  }

  public Command runTrajectory(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public Command pathfindToTrajectory(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(path, kPathConstraints);
  }
}
