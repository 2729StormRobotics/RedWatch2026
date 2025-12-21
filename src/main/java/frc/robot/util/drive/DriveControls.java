package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Centralized control configuration for driver and operator inputs.
 * Maps physical controller inputs to logical control actions.
 */
public class DriveControls {
  // Controllers
  /** Translator joystick for forward/strafe movement */
  public static final CommandJoystick m_translator = new CommandJoystick(0);

  /** Rotator joystick for rotation control */
  public static final CommandJoystick m_rotator = new CommandJoystick(1);

  /** Weapons/operator controller for subsystem controls */
  public static final CommandXboxController m_weaponsController = new CommandXboxController(0);

  // Useful for things that don't need to be triggered
  /** Empty trigger that never fires */
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);

  /** Empty double supplier that always returns 0.0 */
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  // Drive controls
  /** Forward/backward drive input */
  public static DoubleSupplier DRIVE_FORWARD;

  /** Left/right strafe drive input */
  public static DoubleSupplier DRIVE_STRAFE;

  /** Rotation drive input */
  public static DoubleSupplier DRIVE_ROTATE;

  /** Trigger to enable slow mode */
  public static Trigger DRIVE_SLOW;

  /** Trigger to stop drive and reset gyro */
  public static Trigger DRIVE_STOP;

  /** Trigger to hold stop position */
  public static Trigger DRIVE_HOLD_STOP;

  // Drive modes
  /** Trigger to enable robot-relative drive mode */
  public static Trigger DRIVE_ROBOT_RELATIVE;

  /** Trigger to enable field-relative drive mode */
  public static Trigger DRIVE_FIELD_RELATIVE;

  /** Trigger to reset gyro */
  public static Trigger RESET_GYRO;

  // SysId controls
  /** Trigger for quasistatic forward test */
  public static Trigger QUASISTATIC_FORWARD;

  /** Trigger for quasistatic reverse test */
  public static Trigger QUASISTATIC_REVERSE;

  /** Trigger for dynamic forward test */
  public static Trigger DYNAMIC_FORWARD;

  /** Trigger for dynamic reverse test */
  public static Trigger DYNAMIC_REVERSE;

  /**
   * Configures all controls based on the current driver and operator settings.
   * This method should be called during robot initialization.
   */
  public static void configureControls() {
    switch (Constants.driver) {
      case KRITHIK:
        // Driver controls - Krithik's configuration
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX());
        DRIVE_ROTATE = () -> (m_rotator.getTwist() / 2.0);
        RESET_GYRO = m_translator.button(12);

        // Driver settings
        DRIVE_SLOW = m_translator.button(1);
        DRIVE_STOP = m_translator.button(2);
        DRIVE_HOLD_STOP = m_translator.button(3);

        // Driver modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4);
        break;

      case PROGRAMMERS:
      default:
        // Driver controls - Default/programmer configuration
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX());
        DRIVE_ROTATE = () -> (-m_translator.getTwist());
        RESET_GYRO = m_translator.button(12);

        // Driver settings
        DRIVE_SLOW = m_translator.button(1);
        DRIVE_STOP = m_translator.button(2);
        DRIVE_HOLD_STOP = m_translator.button(3);

        // Driver modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4);
        break;
    }
  }
}
