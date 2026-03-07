package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;

import java.util.function.Supplier;
import frc.robot.util.SparkIdleModeTuner;

public class HopperIOReal implements HopperIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public HopperIOReal() {
    // Use the constant from KickerConstants
    motor = new SparkMax(HopperConstants.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HopperConstants.CURRENT_LIMIT_AMPS)
        .inverted(HopperConstants.MOTOR_INVERTED)
        .voltageCompensation(12.0);
    
    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Apply configuration safely
    tryUntilOk(motor, 5, () -> 
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRotationsPerSec = encoder.getVelocity() / 60.0; // RPM to RPS
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();

    // Allow runtime brake/coast selection for hopper motor.
    SparkIdleModeTuner.syncIdleMode(motor, "Hopper/HopperBrake", IdleMode.kCoast);
  }

   
  @Override
  public void setVoltage(double volts) {
    motor.set(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  // Helper method to retry REV commands (copied from your Hood code)
  private static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }
}