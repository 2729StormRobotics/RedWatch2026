package frc.robot.subsystems.shooter.hood; // Ensure this matches your folder path

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.SparkIdleModeTuner;
import frc.robot.util.misc.LoggedTunableNumber;
import java.util.function.Supplier;

/**
 * Real hardware implementation of the hood motor.
 *
 * Troubleshooting checklist (run in this order when the hood misbehaves):
 *  1. SmartDashboard signals
 *     - Confirm that `Shooter/Hood/CurrentAngle` and `Shooter/Hood/SetpointAngle`
 *       are changing when you run hood commands.
 *     - Confirm that `Hood/Position` is changing when you manually move the hood.
 *
 *  2. CAN + device health
 *     - Check the Spark MAX is present on CAN with the correct ID (`HoodConstants.MOTOR_ID`).
 *     - In REV Hardware Client, verify the motor can spin using the built‑in controls.
 *
 *  3. Encoder sanity
 *     - Watch `Hood/Position` while slowly moving the hood by hand:
 *         fully down  ≈ `HoodConstants.MIN_POSITION_ROTATIONS` (about -37)
 *         fully up    ≈ `HoodConstants.MAX_POSITION_ROTATIONS` (about -1)
 *     - If the sign is flipped or the range is wildly different, fix wiring/inversion
 *       or update `HoodConstants.MIN_POSITION_ROTATIONS` / `MAX_POSITION_ROTATIONS`.
 *
 *  4. Soft limits
 *     - Make sure the mechanical hard‑stops line up with the software limits:
 *         `MIN_POSITION_ROTATIONS` must be “hood all the way down”.
 *         `MAX_POSITION_ROTATIONS` must be “hood all the way up”.
 *     - If the hood stops too early or too late, adjust those constants
 *       instead of bypassing soft limits.
 *
 *  5. PID behavior
 *     - Run a `runPositionCommand(...)` or `setHoodAngle(...)` command.
 *     - Verify the motor moves smoothly to the setpoint and stops.
 *     - If it oscillates or is sluggish, tune `HoodConstants.kP/kI/kD`
 *       (start with only kP, add D if needed, leave I at 0 unless absolutely required).
 *
 *  6. Escape hatch (debug only)
 *     - You can temporarily drive the hood with `setVoltage()` or `setPercent()`
 *       but do NOT leave code that disables PID or soft limits.
 *     - Once debugging is done, always go back to `setPosition` / `setAngle`.
 */
public class HoodIOReal implements HoodIO {
  public final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController positionController;
  
  // Track setpoint locally within the subsystem (rotations)
  private double positionSetpointRotations = 0.0;

  // Tunable PID gains for hood position loop
  private final LoggedTunableNumber kP_tunable =
      new LoggedTunableNumber("Shooter/Hood/kP", HoodConstants.kP);
  private final LoggedTunableNumber kI_tunable =
      new LoggedTunableNumber("Shooter/Hood/kI", HoodConstants.kI);
  private final LoggedTunableNumber kD_tunable =
      new LoggedTunableNumber("Shooter/Hood/kD", HoodConstants.kD);

  public HoodIOReal() {
    motor = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    positionController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast) 
        .smartCurrentLimit(HoodConstants.CURRENT_LIMIT_AMPS)
        .inverted(HoodConstants.MOTOR_INVERTED)
        .voltageCompensation(12.0);

    // Soft limits in motor-rotation units:
    //   MAX_POSITION_ROTATIONS ≈ hood fully up
    //   MIN_POSITION_ROTATIONS ≈ hood fully down
    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(HoodConstants.MAX_POSITION_ROTATIONS)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(HoodConstants.MIN_POSITION_ROTATIONS);

    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Using PID constants from HoodConstants
        .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
        .allowedClosedLoopError(0.1, ClosedLoopSlot.kSlot0);

    // Absolute encoder on data port (21T) - used by Turret for CRT multi-turn positioning
    config.absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Zero the hood on startup
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
    
    // Initial target: park somewhere safely inside the motion range
    // this.setPosition((HoodConstants.MIN_POSITION_ROTATIONS + HoodConstants.MAX_POSITION_ROTATIONS) / 2.0);
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }

  @Override
  public void setPosition(double targetRotations) {
    // Clamp to software soft limits so we never command beyond allowed travel
    double clamped =
        MathUtil.clamp(
            targetRotations,
            HoodConstants.MIN_POSITION_ROTATIONS,
            HoodConstants.MAX_POSITION_ROTATIONS);

    this.positionSetpointRotations = clamped;
    positionController.setSetpoint(clamped, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  @Override
  public void setAngle(double angleRadians) {
    // Convert angle (radians) to motor rotations based on configured range.
    double clamped =
        MathUtil.clamp(angleRadians, HoodConstants.MIN_ANGLE_RAD, HoodConstants.MAX_ANGLE_RAD);

    double normalizedAngle =
        (clamped - HoodConstants.MIN_ANGLE_RAD)
            / (HoodConstants.MAX_ANGLE_RAD - HoodConstants.MIN_ANGLE_RAD);

    double targetRotations =
        HoodConstants.MIN_POSITION_ROTATIONS
            + normalizedAngle
                * (HoodConstants.MAX_POSITION_ROTATIONS - HoodConstants.MIN_POSITION_ROTATIONS);

    setPosition(targetRotations);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
  }
  
  @Override
  public void setVoltage(double volts) {
    // Debug helper: still respects SparkMax soft limits.
    motor.setVoltage(volts);
    // Keep the software setpoint roughly in sync for logging
    this.positionSetpointRotations = getPosition();
  }
  public boolean isAtPosition(double target) {
    return Math.abs(getPosition() - target) < 1.0; 
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void stop() {
    this.positionSetpointRotations = getPosition();
    motor.stopMotor();
  }
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double pos = encoder.getPosition();
    inputs.motorPositionRotations = pos;
    inputs.motorVelocityRotationsPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperatureCelsius = motor.getMotorTemperature();

    // Map motor rotations [MIN_POSITION_ROTATIONS, MAX_POSITION_ROTATIONS]
    // to normalized [0, 1] corresponding to [MIN_ANGLE_RAD, MAX_ANGLE_RAD].
    double normalizedPosition =
        (pos - HoodConstants.MIN_POSITION_ROTATIONS)
            / (HoodConstants.MAX_POSITION_ROTATIONS - HoodConstants.MIN_POSITION_ROTATIONS);
    inputs.absolutePositionRotations = MathUtil.clamp(normalizedPosition, 0.0, 1.0);

    // Allow runtime brake/coast selection for hood motor.
    SparkIdleModeTuner.syncIdleMode(motor, "Shooter/HoodBrake", IdleMode.kCoast);

    // PID tuning from Elastic / SmartDashboard when in tuning mode.
    if (Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          this.hashCode(),
          values -> {
            double p = values[0];
            double i = values[1];
            double d = values[2];

            SparkMaxConfig cfg = new SparkMaxConfig();
            cfg.closedLoop.pid(p, i, d);
            tryUntilOk(
                motor,
                5,
                () ->
                    motor.configure(
                        cfg,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters));
          },
          kP_tunable, kI_tunable, kD_tunable);
    }
  }

}