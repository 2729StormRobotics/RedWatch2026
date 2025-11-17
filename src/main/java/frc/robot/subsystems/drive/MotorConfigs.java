package frc.robot.subsystems.drive;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorIdleMode;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorIdleMode;

public final class MotorConfigs {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        drivingConfig
                .idleMode(kDrivingMotorIdleMode)
                .smartCurrentLimit(kDrivingMotorCurrentLimit)
                .signals.primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));
        drivingConfig.encoder
                .positionConversionFactor(kDrivingEncoderPositionFactor) // meters
                .velocityConversionFactor(kDrivingEncoderVelocityFactor); // meters per second
        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

        turningConfig
                .idleMode(kTurningMotorIdleMode)
                .smartCurrentLimit(kTurningMotorCurrentLimit)
                .signals.primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));
        turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(ModuleConstants.kTurningEncoderInverted)
                .positionConversionFactor(kTurningEncoderPositionFactor) // radians
                .velocityConversionFactor(kTurningEncoderVelocityFactor); // radians per second
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, turningFactor);
    }
}
