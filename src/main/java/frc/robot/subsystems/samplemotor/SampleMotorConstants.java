package frc.robot.subsystems.samplemotor;

/**
 * Constants for the sample motor subsystem.
 */
public final class SampleMotorConstants {
  /** CAN ID for the sample motor */
  public static final int MOTOR_CAN_ID = 20; // Example CAN ID - update as needed

  /** Current limit for the motor in amps */
  public static final int CURRENT_LIMIT_AMPS = 40;

  /** Whether the motor is inverted */
  public static final boolean MOTOR_INVERTED = false;

  /** Gear ratio of the motor (motor rotations per output rotation) */
  public static final double GEAR_RATIO = 1.0;

  /** Moment of inertia in kg*m^2 (for simulation) */
  public static final double MOI_KG_M2 = 0.01;

  /** Position conversion factor (rotations per encoder count) */
  public static final double POSITION_CONVERSION_FACTOR = 1.0;

  /** Velocity conversion factor (rotations per second per encoder count per 100ms) */
  public static final double VELOCITY_CONVERSION_FACTOR = 1.0 / 60.0; // Convert RPM to RPS

  private SampleMotorConstants() {}
}

