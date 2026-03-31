package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.util.misc.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {
    private final double k_turretRingTeeth = 200.0;
    private final double k_gear19 = 19.0;
    private final double k_gear21 = 21.0;
    private final double k_gearboxRatio = 4.0;

    private final double k_enc19Offset = 0.0;
    private final double k_enc21Offset = 0.0;

    private final SparkMax motor;
    private final SparkMax auxSpark;
    private final SparkAbsoluteEncoder encoder19;
    private final SparkAbsoluteEncoder encoder21;
    private final RelativeEncoder internalEncoder;

    private final TrapezoidProfile.Constraints m_constraints = 
        new TrapezoidProfile.Constraints(500.0, 600.0);
    private final ProfiledPIDController m_pidController = 
        new ProfiledPIDController(kP, kI, kD, m_constraints);

    private double targetAngleDegrees = 0.0;
    private boolean isClosedLoop = false;
    private double lastAbsoluteAngleDeg = 0.0;
    private boolean initializedFromCrt = false;

    private final LoggedTunableNumber kP_tunable = new LoggedTunableNumber("Shooter/Turret/kP", kP);
    private final LoggedTunableNumber kI_tunable = new LoggedTunableNumber("Shooter/Turret/kI", kI);
    private final LoggedTunableNumber kD_tunable = new LoggedTunableNumber("Shooter/Turret/kD", kD);

    public TurretIOReal(SparkMax auxSparkMax) {
        motor = new SparkMax(12, MotorType.kBrushless);
        auxSpark = auxSparkMax;
        encoder19 = motor.getAbsoluteEncoder();
        encoder21 = auxSpark.getAbsoluteEncoder();
        internalEncoder = motor.getEncoder();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);

        double totalGearRatio = k_gearboxRatio * (k_turretRingTeeth / k_gear19);
        double positionFactor = 360.0 / totalGearRatio;
        motorConfig.encoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(positionFactor / 60.0);

    motorConfig.absoluteEncoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Hardware Soft Limits map to Convention [-180, 180] (Hardware = Convention + 90)
    motorConfig.softLimit
        .forwardSoftLimit(270.0)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-90.0)
        .reverseSoftLimitEnabled(true);

        tryUntilOk(motor, 5, () -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        m_pidController.setTolerance(1.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        ifOk(motor, encoder19::getPosition, (val) -> inputs.absoluteEncoder19Pos = val);
        ifOk(auxSpark, encoder21::getPosition, (val) -> inputs.absoluteEncoder21Pos = val);

    // CRT removed per user request. 
    inputs.crtError = 0.0;

        final double conventionOffset = TurretConstants.FORWARD_OFFSET_DEG;
        ifOk(motor, internalEncoder::getPosition, (val) -> {
            inputs.motorPositionDeg = val + conventionOffset;
            inputs.absoluteAngleDeg = inputs.motorPositionDeg;
            lastAbsoluteAngleDeg = val; 
        });
        
        ifOk(motor, internalEncoder::getVelocity, (val) -> inputs.motorVelocityDegPerSec = val);

    if (!initializedFromCrt) {
      // We assume the robot starts physically at -90 degrees convention (0 degrees hardware).
      internalEncoder.setPosition(0.0);
      double conventionDeg = -90.0;
      m_pidController.reset(conventionDeg);
      inputs.motorPositionDeg = conventionDeg;
      inputs.absoluteAngleDeg = conventionDeg;
      lastAbsoluteAngleDeg = 0.0;
      initializedFromCrt = true;
    }
    
    ifOk(motor, new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage}, 
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (val) -> inputs.currentAmps = val);
    ifOk(motor, motor::getMotorTemperature, (val) -> inputs.temperatureCelsius = val);

    // Allow runtime brake/coast selection for turret motor.
    // SparkIdleModeTuner.syncIdleMode(motor, "Shooter/TurretBrake", IdleMode.kBrake);

    // Turret PID tuning from Elastic / SmartDashboard when in tuning mode.
    if (Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          this.hashCode(),
          values -> {
            double p = values[0];
            double i = values[1];
            double d = values[2];
            m_pidController.setPID(p, i, d);
          },
          kP_tunable, kI_tunable, kD_tunable);
    }

    // Software safety guard: if angle ever leaves the allowed command band, stop the turret.
    // Convention frame (0° = forward). Allowed range matches hardware -180° to 180°.
    if (inputs.motorPositionDeg < TurretConstants.MIN_ANGLE_DEG
        || inputs.motorPositionDeg > TurretConstants.ALLOWED_MAX_DEG) {
      isClosedLoop = false;
      motor.stopMotor();
      return;
    }

    // Run Profiled PID calculation if in closed loop mode (both in convention: 0° = forward)
    if (isClosedLoop) {
      double output = m_pidController.calculate(inputs.motorPositionDeg, targetAngleDegrees);
      motor.set(MathUtil.clamp(output, -.8, 0.8));
    }
  }

  @Override
  public void setAngle(double degrees) {
    // Convention: 0° = forward, CCW positive. Clamp to allowed range (maps to hardware -90° to 180°).
    double clampedDegrees =
        MathUtil.clamp(degrees, TurretConstants.MIN_ANGLE_DEG, TurretConstants.ALLOWED_MAX_DEG);

    if (!isClosedLoop) {
      // Sync internal encoder (hardware) and PID (convention) before starting closed loop
      double conventionDeg = MathUtil.inputModulus(lastAbsoluteAngleDeg + TurretConstants.FORWARD_OFFSET_DEG, -180.0, 180.0);
      m_pidController.reset(conventionDeg);
      isClosedLoop = true;
    }
    targetAngleDegrees = clampedDegrees;
  }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        isClosedLoop = false;
        motor.stopMotor();
    }

    @Override
    public double getDesiredAngle() { return targetAngleDegrees; }
}