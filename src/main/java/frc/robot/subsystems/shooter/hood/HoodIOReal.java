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
import java.util.function.Supplier;
import frc.robot.util.SparkIdleModeTuner;

public class HoodIOReal implements HoodIO {
  public final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController positionController;
  
  
  // Track setpoint locally within the subsystem
  private double positionSetpointRotations = 0.0;

  public HoodIOReal() {
    motor = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    positionController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast) 
        .smartCurrentLimit(HoodConstants.CURRENT_LIMIT_AMPS)
        .inverted(HoodConstants.MOTOR_INVERTED) // Fixed name
        .voltageCompensation(12.0);

    // SOFT LIMITS (forward=-1 = max angle, reverse=-37 = min angle)
    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(-1.0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-37.0);

    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Using PID constants from HoodConstants
        .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

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
    
    // Initial target
    // this.positionSetpointRotations = 15;
    this.setPosition(-15);
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }

  @Override
  public void setPosition(double targetRotations) {
    this.positionSetpointRotations = targetRotations;
    positionController.setSetpoint(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  @Override
  public void setAngle(double angleRadians) {
    // Convert angle (radians) to motor rotations. Hardware mapping: 0° -> -37, 30° -> -1
    double clamped = MathUtil.clamp(angleRadians, HoodConstants.MIN_ANGLE_RAD, HoodConstants.MAX_ANGLE_RAD);
    double targetRotations = -37.0 + (clamped / HoodConstants.MAX_ANGLE_RAD) * 36.0;
    setPosition(targetRotations);
  }

  @Override
  public void setPercent(double percent) {
    motor.set(percent);
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
    // Map motor rotations [-37, -1] to normalized [0, 1] for angle 0° to 30°
    inputs.absolutePositionRotations = MathUtil.clamp((pos + 37.0) / 36.0, 0.0, 1.0);

    // Allow runtime brake/coast selection for hood motor.
    SparkIdleModeTuner.syncIdleMode(motor, "Shooter/HoodBrake", IdleMode.kCoast);
  }

}