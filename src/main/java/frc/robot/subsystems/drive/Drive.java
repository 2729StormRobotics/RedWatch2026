package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.driveBaseRadius;
import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.drive.DriveConstants.pigeonCanId;
import static frc.robot.subsystems.drive.DriveConstants.ppConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.drive.AllianceFlipUtil;
import java.io.File;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Swerve drive built on YAGSL (JSON config in deploy/swerve) and YAMS (the swerve implementation).
 *
 * <p>Module hardware, gearing, offsets and gains live in {@code deploy/swerve}. This class keeps
 * the same public surface the rest of the robot code already used, so callers did not change.
 *
 * <p>The gyro is declared {@code "custom"} in swervedrive.json and supplied here: YAGSL's own
 * Pigeon wiring hands YAMS an angle supplier where the YAMS release we pin expects a
 * Rotation3d supplier.
 */
public class Drive extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  // Raw gyro signals. Null off the real robot, where YAMS simulates heading itself.
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private boolean gyroConnected = false;

  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro.", AlertType.kError);

  private boolean poseInitialized = false;

  // --- ASYMMETRIC RATE LIMITERS ---
  // Positive rate = Acceleration, Negative rate = Deceleration
  // Deceleration is set to -100.0 to make it virtually instant.
  private final SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(3.5, -100.0, 0.0);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10.0, -100.0, 0.0);

  public Drive() {
    if (RobotBase.isReal()) {
      pigeon = new Pigeon2(pigeonCanId);
      pigeon.getConfigurator().setYaw(0.0);
      yaw = pigeon.getYaw();
      yawVelocity = pigeon.getAngularVelocityZWorld();
      yaw.setUpdateFrequency(100.0);
      yawVelocity.setUpdateFrequency(100.0);
    } else {
      pigeon = null;
      yaw = null;
      yawVelocity = null;
    }

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve"));
    SwerveDriveConfig config =
        new SwerveDriveConfig(this)
            .withTelemetry("Swerve", TelemetryVerbosity.LOW)
            .withGyro(this::getGyroRotation3d)
            .withMaximumChassisSpeed(
                MetersPerSecond.of(maxSpeedMetersPerSec),
                RadiansPerSecond.of(maxSpeedMetersPerSec / driveBaseRadius));
    swerveDrive = SwerveParser.createSwerveDrive(config);

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

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  /** Heading supplied to YAMS: yaw only, the same signal the old gyro IO used. */
  private Rotation3d getGyroRotation3d() {
    if (yaw == null) {
      return new Rotation3d();
    }
    return new Rotation3d(0.0, 0.0, Units.degreesToRadians(yaw.getValueAsDouble()));
  }

  @Override
  public void periodic() {
    if (pigeon != null) {
      gyroConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
      Logger.recordOutput("Drive/Gyro/Connected", gyroConnected);
      Logger.recordOutput("Drive/Gyro/YawPosition", Rotation2d.fromDegrees(yaw.getValueAsDouble()));
      Logger.recordOutput("Drive/Gyro/YawVelocityRadPerSec", getYawVelocityRadPerSec());
    }
    gyroDisconnectedAlert.set(pigeon != null && !gyroConnected);

    if (!poseInitialized && DriverStation.getAlliance().isPresent()) {
      setPose(AllianceFlipUtil.apply(Pose2d.kZero));
      poseInitialized = true;
    }

    if (DriverStation.isDisabled()) {
      swerveDrive.stop();
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Odometry only advances when this is called.
    swerveDrive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive.simIterate();
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
          limitedRot);
    }

    // YAMS discretizes and desaturates the speeds itself.
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", targetSpeeds);
    Logger.recordOutput(
        "SwerveStates/Setpoints", swerveDrive.getStateFromRobotRelativeChassisSpeeds(targetSpeeds));
    swerveDrive.setRobotRelativeChassisSpeeds(targetSpeeds);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", swerveDrive.getDesiredModuleStates());
  }

  /** Runs the drive motors at the given voltage with every module pointed forward. */
  public void runCharacterization(double output) {
    for (SwerveModule module : swerveDrive.getModules()) {
      module.getDriveMotorController().setVoltage(Volts.of(output));
      module.getAzimuthMotorController().setPosition(Rotations.of(0.0));
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    swerveDrive.lockPose();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return swerveDrive.getModuleStates();
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getRobotRelativeSpeed();
  }

  /** Wheel positions in radians, for wheel radius characterization. */
  public double[] getWheelRadiusCharacterizationPositions() {
    SwerveModule[] modules = swerveDrive.getModules();
    double[] values = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      values[i] = modules[i].getDriveMotorController().getMechanismPosition().in(Radians);
    }
    return values;
  }

  /** Average wheel velocity in rad/s, for feedforward characterization. */
  public double getFFCharacterizationVelocity() {
    SwerveModule[] modules = swerveDrive.getModules();
    double output = 0.0;
    for (SwerveModule module : modules) {
      output += module.getDriveMotorController().getMechanismVelocity().in(RadiansPerSecond) / modules.length;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @AutoLogOutput(key = "Odometry/RobotAlliance")
  public Pose2d getAlliancePose() {
    return AllianceFlipUtil.apply(getPose());
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current gyro yaw velocity in radians per second. */
  public double getYawVelocityRadPerSec() {
    if (yawVelocity == null) {
      return getChassisSpeeds().omegaRadiansPerSecond;
    }
    return Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  /** Resets the current odometry pose (YAMS also re-references the gyro heading to it). */
  public void setPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void resetYaw() {
    Rotation2d newHeading = AllianceFlipUtil.shouldFlip() ? Rotation2d.fromDegrees(180) : Rotation2d.kZero;
    setPose(new Pose2d(getPose().getTranslation(), newHeading));
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
