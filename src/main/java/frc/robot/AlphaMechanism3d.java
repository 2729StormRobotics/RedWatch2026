package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.MechanismConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Master 3D mechanism visualization for Project Titan.
 * Logic is Robot-Relative to prevent double-transforming in AdvantageScope.
 */
public class AlphaMechanism3d {
  private static AlphaMechanism3d instance;

  public static AlphaMechanism3d getInstance() {
    if (instance == null) {
      instance = new AlphaMechanism3d();
    }
    return instance;
  }

  // Subsystem States
  private Rotation2d turretAngle = new Rotation2d();
  private Rotation2d hoodAngle = new Rotation2d();
  private Rotation2d intakeAngle = new Rotation2d();
  private double climbExtensionMeters = 0.0;

  public void setShooter(Rotation2d turret, Rotation2d hood) {
    this.turretAngle = turret;
    this.hoodAngle = hood;
  }

  public void setIntake(Rotation2d intake) {
    this.intakeAngle = intake;
  }

  public void setClimb(double meters) {
    this.climbExtensionMeters = meters;
  }

  /**
   * Logs all component poses relative to Robot Origin (0,0,0).
   */
  public void log() {
    Pose3d robotRoot = new Pose3d();

    // 1. Turret & Hood
    Pose3d turretPose = robotRoot
        .transformBy(MechanismConstants.robotToTurret)
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians())));

    Pose3d hoodPose = turretPose
        .transformBy(MechanismConstants.turretToHood)
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(-hoodAngle.getRadians(), 0, 0)));

    // 2. Intake Pivot
    // Retracted is 0 rotations (pointing down/in), Deployed is 1.0 rotations (out).
    // Pose3d intakePose = robotRoot
    //     .transformBy(MechanismConstants.robotToIntakePivot)
    //     .transformBy(new Transform3d(new Translation3d(), new Rotation3d(intakeAngle.getRadians(), 0, 0)));

    // 3. Linked Hopper (254-style floor extension)
    // The hopper floor slides out as a function of the intake pivot angle.
    // If intake is 1.0 rotations (deployed), the hopper is fully extended.
    double extensionPercent = Math.min(1.0, Math.max(0.0, intakeAngle.getRotations())); 
    double hopperExtensionX = extensionPercent * MechanismConstants.kMaxHopperExtensionMeters*3;

    Pose3d hopperPose = robotRoot
        .transformBy(MechanismConstants.robotToHopperBase)
        .transformBy(new Transform3d(new Translation3d(0, -hopperExtensionX, 0), new Rotation3d()));

    // 4. Climb
    Pose3d climbPose = robotRoot
        .transformBy(MechanismConstants.robotToClimbBase)
        .transformBy(new Transform3d(new Translation3d(0, 0, climbExtensionMeters), new Rotation3d()));

    // Log to AdvantageScope "Components"
    Logger.recordOutput("Mechanisms/ComponentPoses", new Pose3d[] {
      turretPose, 
      hoodPose, 
    //   intakePose, 
      hopperPose, 
      climbPose
    });
  }
}