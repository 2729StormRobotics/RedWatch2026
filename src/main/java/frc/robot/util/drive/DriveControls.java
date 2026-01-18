package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
  public static final CommandGenericHID m_translator;

  /** Rotator joystick for rotation control */
  public static final CommandGenericHID m_rotator;

  // Static initializer to set up controllers based on mode
  static {
    if (Constants.currentMode == Constants.Mode.REAL) {
      // Real mode: use CommandJoystick
      m_translator = new CommandJoystick(0);
      m_rotator = new CommandJoystick(1);
    } else {
      // Simulation mode: use CommandGenericHID
      m_translator = new CommandGenericHID(0);
      m_rotator = new CommandGenericHID(1);
    }
  }

  /**
   * Gets the Y axis value from a controller.
   * Works with both CommandJoystick (using getY()) and CommandGenericHID (using getRawAxis(1)).
   * Both should return negative when pushed forward (standard joystick convention).
   */
  private static double getY(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getY();
    } else {
      // getRawAxis(1) returns the raw Y axis value (negative when pushed forward)
      return -controller.getHID().getRawAxis(1);
    }
  }

  /**
   * Gets the X axis value from a controller.
   * Works with both CommandJoystick (using getX()) and CommandGenericHID (using getRawAxis(0)).
   * Both should return negative when pushed right (standard joystick convention).
   */
  private static double getX(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getX();
    } else {
      // getRawAxis(0) returns the raw X axis value (negative when pushed right)
      return -controller.getHID().getRawAxis(0);
    }
  }

  /**
   * Gets the twist axis value from a controller.
   * Works with both CommandJoystick (using getTwist()) and CommandGenericHID (using getRawAxis(2)).
   * Both should return the raw twist/rotation axis value.
   */
  private static double getTwist(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getTwist();
    } else {
      // getRawAxis(2) returns the raw twist/rotation axis value
      return -m_translator.getHID().getRawAxis(2);
    }
  }

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
        DRIVE_FORWARD = () -> (-getY(m_rotator));
        DRIVE_STRAFE = () -> (-getX(m_rotator));
        DRIVE_ROTATE = () -> (getTwist(m_rotator) / 2.0);
        RESET_GYRO = m_rotator.button(12);

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
        DRIVE_FORWARD = () -> (-getY(m_translator));
        DRIVE_STRAFE = () -> (-getX(m_translator));
        DRIVE_ROTATE = () -> (-getTwist(m_translator));
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
