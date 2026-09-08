package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

/** Robocon 2026 field, in meters, using the blue-alliance origin. */
public final class FieldConstants {
  public static final AprilTagFieldLayout aprilTagLayout = loadLayout();
  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  private static AprilTagFieldLayout loadLayout() {
    try {
      return new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath()
              .resolve("2026-robocon-welded-photonvision-wpilib.json"));
    } catch (IOException e) {
      throw new IllegalStateException("Unable to load the Robocon AprilTag field layout", e);
    }
  }

  private FieldConstants() {}
}
