package frc.robot.subsystems.kicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;

/** Real hardware implementation of kickerIO: one NEO Vortex on a SPARK Flex. */
public class kickerIOReal implements kickerIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;

  public kickerIOReal() {
    // Create the motor object. The CAN ID must match what is set on the physical controller.
    motor = new SparkFlex(kickerConstants.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Describe how the motor controller should behave, then send it to the controller below.
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(kickerConstants.CURRENT_LIMIT_AMPS)
        .inverted(kickerConstants.MOTOR_INVERTED)
        .voltageCompensation(12.0);

    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Apply configuration safely (retries a few times in case the CAN message is dropped)
    tryUntilOk(motor, 5, () ->
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(kickerIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRotationsPerSec = encoder.getVelocity() / 60.0; // encoder reports RPM
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.temperatureCelsius = motor.getMotorTemperature();
  }

  @Override
  public void setPercentIO(double percent) {
    // Direction is handled by MOTOR_Inverted in kickerConstants, not here.
    motor.set(percent);
  }

  @Override
  public void setPercentIO(double percent) {
    public void stop() {
    motor.stopMotor();
  }

  // Helper method to retry REV commands: configuration messages over CAN can occasionally fail.
  private static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }
}
