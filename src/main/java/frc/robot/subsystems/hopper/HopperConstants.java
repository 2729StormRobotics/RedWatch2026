// import edu.wpi.first.math.util.Units;
package frc.robot.subsystems.hopper;

public final class HopperConstants {
  /** CAN ID */
  public static final int MOTOR_ID = 16;

  /** Current limit for NEO 550 motors in amps */
  public static final int CURRENT_LIMIT_AMPS = 25;

  /** Whether the motor is inverted */
  public static final boolean MOTOR_INVERTED = false;

  /** Voltages for commands */
  public static final double KICK_VOLTAGE = -0.5;
  public static final double REVERSE_VOLTAGE = 0.5;

  /** Gear ratio */
  public static final double GEAR_RATIO = 25.0;

  /** Simulation constants */
  public static final double MOI_KG_M2 = 0.01;
  public static final double MASS_KG = 0.5;
  public static final double ARM_LENGTH_M = 0.3;
  public static final double MIN_ANGLE_RAD = 0.0;
  public static final double MAX_ANGLE_RAD = 0.0;

  private HopperConstants() {
  }
}