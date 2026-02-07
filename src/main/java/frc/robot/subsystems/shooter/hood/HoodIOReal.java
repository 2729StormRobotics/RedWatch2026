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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;

public class HoodIOReal implements HoodIO {
  private final SparkMax motor;
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
        .idleMode(IdleMode.kBrake) 
        .smartCurrentLimit(HoodConstants.CURRENT_LIMIT_AMPS)
        .inverted(HoodConstants.MOTOR_INVERTED) // Fixed name
        .voltageCompensation(12.0);

    // SOFT LIMITS
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

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Zero the hood on startup
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
    
    // Initial target
    this.positionSetpointRotations = 15;
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }

  public void setPosition(double targetRotations) {
    this.positionSetpointRotations = targetRotations;
    positionController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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

}