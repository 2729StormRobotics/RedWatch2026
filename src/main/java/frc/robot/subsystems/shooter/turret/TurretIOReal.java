package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

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

        // --- SOFT LIMITS ---
        // We subtract FORWARD_OFFSET_DEG because the motor's "0" is actually the offset point.
        // This ensures the hardware-level limits align with your subsystem-level range.
        motorConfig.softLimit
            .forwardSoftLimit(TurretConstants.ALLOWED_MAX_DEG - TurretConstants.FORWARD_OFFSET_DEG)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.MIN_ANGLE_DEG - TurretConstants.FORWARD_OFFSET_DEG)
            .reverseSoftLimitEnabled(true);

        tryUntilOk(motor, 5, () -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        m_pidController.setTolerance(1.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        ifOk(motor, encoder19::getPosition, (val) -> inputs.absoluteEncoder19Pos = val);
        ifOk(auxSpark, encoder21::getPosition, (val) -> inputs.absoluteEncoder21Pos = val);

        double[] crtResult = calculateCrtAngle(inputs.absoluteEncoder19Pos, inputs.absoluteEncoder21Pos);
        inputs.crtError = crtResult[1];

        final double conventionOffset = TurretConstants.FORWARD_OFFSET_DEG;
        ifOk(motor, internalEncoder::getPosition, (val) -> {
            inputs.motorPositionDeg = val + conventionOffset;
            inputs.absoluteAngleDeg = inputs.motorPositionDeg;
            lastAbsoluteAngleDeg = val; 
        });
        
        ifOk(motor, internalEncoder::getVelocity, (val) -> inputs.motorVelocityDegPerSec = val);

        // Initialize position based on CRT logic to ensure we aren't starting at a random offset
        if (!initializedFromCrt && inputs.absoluteEncoder19Pos != 0) {
            double initialAngle = MathUtil.inputModulus(crtResult[0] + TurretConstants.ZERO_OFFSET_DEG, -180.0, 180.0);
            setInternalPosition(initialAngle);
            initializedFromCrt = true;
        }

        if (Constants.tuningMode) {
            LoggedTunableNumber.ifChanged(this.hashCode(), values -> 
                m_pidController.setPID(values[0], values[1], values[2]), 
                kP_tunable, kI_tunable, kD_tunable);
        }

        if (isClosedLoop) {
            double output = m_pidController.calculate(inputs.motorPositionDeg, targetAngleDegrees);
            motor.set(MathUtil.clamp(output, -0.8, 0.8));
        }
    }

    @Override
    public void setInternalPosition(double degrees) {
        // Adjusts the internal encoder so that its current physical position 
        // maps to the 'degrees' provided in your subsystem convention.
        internalEncoder.setPosition(degrees - TurretConstants.FORWARD_OFFSET_DEG);
        m_pidController.reset(degrees);
    }

    private static double frac(double x) {
        double f = x - Math.floor(x);
        return (f >= 1.0 - 1e-12) ? 0.0 : f;
    }

    private static double circularError(double a, double b) {
        double diff = Math.abs(a - b);
        return (diff > 0.5) ? (1.0 - diff) : diff;
    }

    private double[] calculateCrtAngle(double raw19, double raw21) {
        double r19 = frac(raw19 - k_enc19Offset);
        double r21 = frac(raw21 - k_enc21Offset);
        double bestError = Double.MAX_VALUE;
        double bestTurretDegrees = 0.0;
        final double inv19 = k_gear19 / k_turretRingTeeth;
        final double ratio19to21 = k_gear19 / k_gear21;

        for (int k = -15; k <= 15; k++) {
            double totalRotations19 = k + r19;
            double turretRotations = totalRotations19 * inv19;
            double expectedR21 = frac(totalRotations19 * ratio19to21);
            double error = circularError(r21, expectedR21);
            double candidateDegrees = -turretRotations * 360.0;

            if (error < bestError) {
                bestError = error;
                bestTurretDegrees = candidateDegrees;
            }
        }
        return new double[] {MathUtil.inputModulus(bestTurretDegrees, -180.0, 180.0), bestError};
    }

    @Override
    public void setAngle(double degrees) {
        if (!isClosedLoop) {
            m_pidController.reset(lastAbsoluteAngleDeg + TurretConstants.FORWARD_OFFSET_DEG);
            isClosedLoop = true;
        }
        targetAngleDegrees = degrees;
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