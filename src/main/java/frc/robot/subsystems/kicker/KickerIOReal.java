package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class KickerIOReal implements KickerIO {
  private final SparkMax motor;

  public KickerIOReal() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(MOTOR_INVERTED)
        .voltageCompensation(12.0);
    tryUntilOk(motor, 5, () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}
