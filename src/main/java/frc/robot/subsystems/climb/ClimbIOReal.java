package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ClimbIOReal implements ClimbIO {
  private final SparkMax motor;
  // Lock mechanism would be a solenoid or servo - simplified here

  public ClimbIOReal() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(MOTOR_INVERTED)
        .voltageCompensation(12.0);
    tryUntilOk(motor, 5, () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
    inputs.lockEngaged = false; // Would read from actual lock sensor
    // TODO: Convert encoder position to meters based on actual mechanism geometry
    // For now, using a placeholder conversion (adjust based on actual gear ratio and lead screw pitch)
    ifOk(motor, this::getPosition, (value) -> {
      // Assuming encoder rotations, convert to meters
      // This is a placeholder - adjust based on actual mechanism
      inputs.positionMeters = value * 0.01; // Placeholder: 0.01 m per rotation
    });
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setLock(boolean engaged) {
    // Would control solenoid/servo here
  }

  public double getPosition() {
    return 0;
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}
