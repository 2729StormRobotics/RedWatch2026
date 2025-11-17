package frc.robot.subsystems.SUBSYSTEMNAME;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Replace with your motor library imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Hardware implementation template for any subsystem using SparkMax motors.
 * Replace SUBSYSTEMNAME with your specific subsystem name.
 */
public class SubsystemIOSparkMax implements SubsystemIO {

    // Motor(s) and config
    private SparkMax motor;
    private SparkMaxConfig motorConfig;

    // Optional limit switch or sensor
    // private SparkLimitSwitch objectDetector;

    public SubsystemIOSparkMax(int motorPort) {
        motor = new SparkMax(motorPort, MotorType.kBrushless);
        // objectDetector = motor.getForwardLimitSwitch();

        // Configure motor
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.pid(0.0, 0.0, 0.0); // Replace with PID constants
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(40);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        stop();
    }

    /** Returns true if object/sensor is triggered */
    @Override
    public boolean hasObject() {
        // Example: return objectDetector.isPressed();
        return false;
    }

    /** Stop the motor */
    @Override
    public void stop() {
        motor.set(0);
    }

    /** Run motor forward (user-defined) */
    @Override
    public void runForward() {
        motor.set(0.5); // Replace with desired forward speed
    }

    /** Run motor in reverse (user-defined) */
    @Override
    public void runReverse() {
        motor.set(-0.5); // Replace with desired reverse speed
    }

    /** Optional: reverse direction of current output */
    @Override
    public void toggleDirection() {
        motor.set(-motor.get());
    }

    /** Optional command for forward motion */
    @Override
    public Command intakeCommand() {
        return new InstantCommand(() -> runForward());
    }

    /** Optional command for reverse motion */
    @Override
    public Command outtakeCommand() {
        return new InstantCommand(() -> runReverse());
    }

    /** Update telemetry / inputs */
    @Override
    public void updateInputs(SubsystemIOInputs inputs) {
        inputs.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.positionRad = 0; // Replace with encoder position if available
        inputs.positionDegrees = 0; // Replace with encoder degrees if available
        inputs.velocityRadPerSec = motor.getEncoder().getVelocity();
    }
}
