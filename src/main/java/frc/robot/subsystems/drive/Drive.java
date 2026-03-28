// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.driveBaseRadius;
import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;
import static frc.robot.subsystems.drive.DriveConstants.ppConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.drive.AllianceFlipUtil;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
  private boolean poseInitialized = false;

  // --- ASYMMETRIC RATE LIMITERS ---
  // Positive rate = Acceleration, Negative rate = Deceleration
  // Deceleration is set to -100.0 to make it virtually instant.
  private final SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(3.5, -100.0, 0.0);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10.0, -100.0, 0.0);

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

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
    SparkOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity, 
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(3.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    sysId = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    if (!poseInitialized && DriverStation.getAlliance().isPresent()) {
      setPose(AllianceFlipUtil.apply(Pose2d.kZero));
      poseInitialized = true;
    }

    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) { module.periodic(); }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) { module.stop(); }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = Math.min(sampleTimestamps.length, gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE);
    
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = speeds;

    if (DriverStation.isTeleopEnabled()) {
      Translation2d linearVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      double magnitude = linearVelocity.getNorm();
      Rotation2d direction = magnitude > 1e-6 ? linearVelocity.getAngle() : Rotation2d.kZero;

      // The limiter now allows instant deceleration but smooth acceleration
      double limitedMagnitude = magnitudeLimiter.calculate(magnitude);
      double limitedRot = rotLimiter.calculate(speeds.omegaRadiansPerSecond);

      targetSpeeds = new ChassisSpeeds(
          direction.getCos() * limitedMagnitude,
          direction.getSin() * limitedMagnitude,
          limitedRot
      );
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(targetSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
    
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) { modules[i].runCharacterization(output); }
  }

  public void stop() { runVelocity(new ChassisSpeeds()); }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) { headings[i] = moduleTranslations[i].getAngle(); }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) { states[i] = modules[i].getState(); }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) { states[i] = modules[i].getPosition(); }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() { return kinematics.toChassisSpeeds(getModuleStates()); }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) { values[i] = modules[i].getWheelRadiusCharacterizationPosition(); }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) { output += modules[i].getFFCharacterizationVelocity() / 4.0; }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); }

  @AutoLogOutput(key = "Odometry/RobotAlliance")
  public Pose2d getAlliancePose() { return AllianceFlipUtil.apply(getPose()); }

  public Rotation2d getRotation() { return getPose().getRotation(); }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void resetYaw() {
    gyroIO.resetYaw();
    Rotation2d newHeading = AllianceFlipUtil.shouldFlip() ? Rotation2d.fromDegrees(180) : Rotation2d.kZero;
    setPose(new Pose2d(getPose().getX(), getPose().getY(), newHeading));
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() { return maxSpeedMetersPerSec; }
  public double getMaxAngularSpeedRadPerSec() { return maxSpeedMetersPerSec / driveBaseRadius; }
}