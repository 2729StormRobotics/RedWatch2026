package frc.robot.subsystems.vision;

/**
 * Constants for the vision subsystem.
 */
public final class VisionConstants {
  /** NetworkTables name for the left Limelight camera */
  public static final String LEFT_LIMELIGHT_NAME = "limelight-left";

  /** NetworkTables name for the right Limelight camera */
  public static final String RIGHT_LIMELIGHT_NAME = "limelight-right";

  /** Maximum valid distance to trust a single tag (meters) */
  public static final double MAX_SINGLE_TAG_DISTANCE = 4.0;

  /** Minimum standard deviation for X/Y translation when multiple tags are detected (meters) 0.5 */ 
  public static final double MULTI_TAG_STD_DEV = 0.15;

  /** Maximum standard deviation for X/Y translation when single tag is far (meters) */
  public static final double FAR_SINGLE_TAG_STD_DEV = 2.0;

  /** Standard deviation for X/Y translation when single tag is close (meters) 0.8 */
  public static final double CLOSE_SINGLE_TAG_STD_DEV = 0.5;

  /** Standard deviation for rotation (set to infinity to trust gyro only) */
  public static final double ROTATION_STD_DEV = Double.POSITIVE_INFINITY;

  /** Maximum age of vision measurement to accept (seconds) */
  public static final double MAX_MEASUREMENT_AGE = 0.5;

  /** Minimum number of tags required for high-confidence measurement */
  public static final int MIN_TAGS_FOR_HIGH_CONFIDENCE = 2;

  /** Throttle value when robot is disabled (100-200 recommended for thermal management) */
  public static final int THROTTLE_DISABLED = 200;

  /** Throttle value when robot is enabled (0 = full speed) */
  public static final int THROTTLE_ENABLED = 0;

  /** IMU mode for MegaTag 2 (0=external only, 1=seed internal, 2=use internal for MT2) */
  public static final int IMU_MODE = 1; // Recommended: use internal IMU for best performance

  /** Maximum angular velocity to accept vision updates (degrees per second) */
  public static final double MAX_ACCEPTABLE_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;

  private VisionConstants() {}
}

