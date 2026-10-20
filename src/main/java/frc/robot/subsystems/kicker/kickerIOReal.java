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
    // TODO: copy the latest readings from the hardware into the inputs object.
    //   inputs.positionRotations      <- encoder.getPosition()
    //   inputs.velocityRotationsPerSec <- encoder.getVelocity(), but the encoder reports RPM,
    //                                    so convert to rotations per second (divide by 60)
    //   inputs.appliedVolts           <- motor.getAppliedOutput() * motor.getBusVoltage()
    //                                    (applied output is a -1..1 fraction of bus voltage)
    //   inputs.currentAmps            <- motor.getOutputCurrent()
    //
    // Optional: let students flip brake/coast from the dashboard while the robot runs with
    //   SparkIdleModeTuner.syncIdleMode(motor, "Kicker/KickerBrake", IdleMode.kCoast);
    //   (needs: import frc.robot.util.SparkIdleModeTuner;)
  }

  @Override
  public void setPercentIO(double percent) {
    // TODO: tell the motor controller to run at this percent output with motor.set(...).
    //   Heads up: this kicker is mounted so that positive "kick" needs the OPPOSITE motor
    //   direction. Decide whether to negate the value here, or flip MOTOR_INVERTED in
    //   kickerConstants instead - and why one is a better place to do it than the other.
  }

  @Override
  public void stop() {
    // TODO: stop the motor. motor.stopMotor() does the job.
  }

  // Helper method to retry REV commands: configuration messages over CAN can occasionally fail.
  private static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }
}
