package frc.robot.util;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

/**
 * Utility for exposing a per-motor brake/coast toggle to SmartDashboard (and Elastic).
 *
 * <p>Each motor that wants a runtime-selectable idle mode should call
 * {@link #syncIdleMode(SparkBase, String, IdleMode)} periodically. This will:
 *
 * <ul>
 *   <li>Publish a boolean key on SmartDashboard under {@code Motors/...}
 *   <li>Read the desired state from that key
 *   <li>Apply {@link IdleMode#kBrake} when true and {@link IdleMode#kCoast} when false
 *   <li>Avoid redundant CAN traffic by only sending changes
 * </ul>
 *
 * <p>In Elastic, configure a {@code Toggle Switch} widget pointed at the matching
 * NetworkTables topic, for example:
 *
 * <pre>
 * topic: "/SmartDashboard/Motors/Drive/FL/DriveBrake"
 * data_type: "boolean"
 * </pre>
 */
public final class SparkIdleModeTuner {
  private static final String BASE_KEY = "Motors/";
  private static final Map<Integer, IdleMode> lastAppliedById = new HashMap<>();

  private SparkIdleModeTuner() {}

  /**
   * Synchronize a Spark's idle mode with a SmartDashboard / Elastic toggle.
   *
   * @param motor The Spark motor controller.
   * @param name Hierarchical name under {@code Motors/}, for example
   *     {@code "Drive/FL/DriveBrake"} or {@code "Intake/RollerBrake"}.
   * @param defaultMode Idle mode to use when the key is first created.
   */
  public static void syncIdleMode(SparkBase motor, String name, IdleMode defaultMode) {
    if (motor == null || name == null) {
      return;
    }

    String key = BASE_KEY + name;
    boolean defaultBrake = defaultMode == IdleMode.kBrake;

    // Initialize the dashboard key once with the default.
    if (!SmartDashboard.containsKey(key)) {
      SmartDashboard.putBoolean(key, defaultBrake);
    }

    boolean brakeRequested = SmartDashboard.getBoolean(key, defaultBrake);
    IdleMode desiredMode = brakeRequested ? IdleMode.kBrake : IdleMode.kCoast;

    int deviceId = motor.getDeviceId();
    IdleMode previousMode = lastAppliedById.get(deviceId);

    if (previousMode != desiredMode) {
      // Apply minimal config update based on motor type.
      if (motor instanceof SparkMax max) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(desiredMode);
        tryUntilOk(
            max,
            5,
            () -> max.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
      } else if (motor instanceof SparkFlex flex) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(desiredMode);
        tryUntilOk(
            flex,
            5,
            () -> flex.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
      }

      lastAppliedById.put(deviceId, desiredMode);
    }
  }
}

