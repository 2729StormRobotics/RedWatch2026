package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Template subsystem for “<SubsystemName>”. */
public class <SubsystemName> extends SubsystemBase {
    // Hardware declarations (motors, sensors, solenoids, etc.)
    // Example:
    // private final MotorController m_motor = new MotorController(<port>);
    // private final DoubleSensor m_sensor = new DoubleSensor(<port>);

    /** Creates a new <SubsystemName> subsystem. */
    public <SubsystemName>() {
        // Initialize hardware, set defaults, reset sensors, etc.
        // Example: m_motor.setInverted(false);
        // Example: m_sensor.reset();
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
        // Example: update dashboard, read sensors, telemetry
    }

    
	//These are all funcational Commands 

    // Public methods for commands to use:
    /** Move the <SubsystemName> to a pre-set position. */
    public void goToPresetPosition() {
        // e.g., m_motor.set(<velocityOrPosition>);
    }

    /** Stop all motion of the subsystem. */
    public void stop() {
        // e.g., m_motor.stopMotor();
    }

    /** Returns true if the subsystem is in the desired state. */
    public boolean isAtTarget() {
        // e.g., return Math.abs(m_sensor.getPosition() - target) < tolerance;
        return false;
    }

    // Add more methods as needed, e.g. manualControl(double speed),
    // setBrakeMode(boolean enable), zeroEncoder(), etc.
}
