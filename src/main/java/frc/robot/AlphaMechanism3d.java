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

  // Shooter State
  private Rotation2d turretAngle = new Rotation2d();
  private Rotation2d hoodAngle = new Rotation2d();

  // Intake & Hopper State
  private Rotation2d intakeAngle = new Rotation2d();

  // Climb State
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

    // --- SHOOTER HIERARCHY ---
    Pose3d turretPose = robotRoot
        .transformBy(MechanismConstants.robotToTurret)
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians())));

    Pose3d hoodPose = turretPose
        .transformBy(MechanismConstants.turretToHood)
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -hoodAngle.getRadians(), 0)));

    // --- INTAKE & LINKED HOPPER ---
    Pose3d intakePose = robotRoot
        .transformBy(MechanismConstants.robotToIntakePivot)
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, intakeAngle.getRadians(), 0)));

    // Hopper is mechanically linked. When intake is 90 deg (deployed), hopper is max extension.
    // Logic: Extension = Deployed_Pos * sin(Intake_Angle)
    double hopperExtension = MechanismConstants.kMaxHopperExtensionMeters * Math.sin(intakeAngle.getRadians());
    Pose3d hopperPose = robotRoot
        .transformBy(MechanismConstants.robotToHopperBase)
        .transformBy(new Transform3d(new Translation3d(hopperExtension, 0, 0), new Rotation3d()));

    // --- CLIMB ---
    Pose3d climbPose = robotRoot
        .transformBy(MechanismConstants.robotToClimbBase)
        .transformBy(new Transform3d(new Translation3d(0, 0, climbExtensionMeters), new Rotation3d()));

    // Log everything to one array for AdvantageScope "Components"
    Logger.recordOutput("Mechanisms/ComponentPoses", new Pose3d[] {
      turretPose, 
      hoodPose, 
      intakePose, 
      hopperPose, 
      climbPose
    });
  }
}