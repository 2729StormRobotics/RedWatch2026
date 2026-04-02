package frc.robot.util.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Centralized control configuration for driver and operator inputs.
 * Maps physical controller inputs to logical control actions.
 *
 * To change bindings:
 * - Driver axes + basic drive buttons: edit {@link #configureDriverBindings()}.
 * - Weapons / subsystems (hood, intake, shooter, hopper, etc.):
 * edit {@link #configureSubsystemBindings()}.
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
      m_translator = new CommandJoystick(Constants.ElectricalLayout.CONTROLLER_DRIVER_ID);
      m_rotator = new CommandJoystick(Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID);
    } else {
      // Simulation mode: use CommandGenericHID
      m_translator = new CommandGenericHID(Constants.ElectricalLayout.CONTROLLER_DRIVER_ID);
      m_rotator = new CommandGenericHID(Constants.ElectricalLayout.CONTROLLER_OPERATOR_ID);
    }
  }

  /**
   * Gets the Y axis value from a controller.
   * Works with both CommandJoystick (using getY()) and CommandGenericHID (using
   * getRawAxis(1)).
   * Both should return negative when pushed forward (standard joystick
   * convention).
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
   * Works with both CommandJoystick (using getX()) and CommandGenericHID (using
   * getRawAxis(0)).
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
   * Works with both CommandJoystick (using getTwist()) and CommandGenericHID
   * (using getRawAxis(2)).
   * Both should return the raw twist/rotation axis value.
   */
  private static double getTwist(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getHID().getRawAxis(3);
    } else {
      // getRawAxis(2) returns the raw twist/rotation axis value
      return controller.getHID().getRawAxis(2);
    }
  }

  /** Weapons/operator controller for subsystem controls */
  public static final CommandXboxController m_weaponsController = new CommandXboxController(
      Constants.ElectricalLayout.CONTROLLER_WEAPONS_ID);

  private static double applyDriveDeadband(double value) {
    return MathUtil.applyDeadband(value, Constants.OperatorConstants.kDriveDeadband);
  }
  public static Trigger EXTEND_CLIMBER;
  public static Trigger RETRACT_CLIMBER;

  // Drive controls
  /** Forward/backward drive input */
  public static DoubleSupplier DRIVE_FORWARD;

  /** Left/right strafe drive input */
  public static DoubleSupplier DRIVE_STRAFE;

  /** Rotation drive input */
  public static DoubleSupplier DRIVE_ROTATE;

  public static Trigger EXTEND_INTAKE;

  /** Trigger to hold stop position */
  public static Trigger DRIVE_HOLD_STOP;
  public static Trigger agitateTrigger;

  // Drive modes
  /** Trigger to enable robot-relative drive mode */
  public static Trigger DRIVE_ROBOT_RELATIVE;

  /** Trigger to enable field-relative drive mode */
  public static Trigger DRIVE_FIELD_RELATIVE;

  /** Trigger to reset gyro */
  public static Trigger RESET_GYRO;

  /** POV triggers on translator: rotate robot to face this field angle while held */
  public static Trigger POV_UP;
  public static Trigger POV_DOWN;
  public static Trigger POV_LEFT;
  public static Trigger POV_RIGHT;
  /** Diagonal POV (45° intervals) */
  public static Trigger POV_UP_RIGHT;
  public static Trigger POV_DOWN_RIGHT;
  public static Trigger POV_DOWN_LEFT;
  public static Trigger POV_UP_LEFT;

  public static Trigger INTAKE_TRIGGER;
  public static Trigger RETRACT_INTAKE;

  public static Trigger flyWheelTrigger;

  public static Trigger reverseKicker;

  public static Trigger HopperTrigger;
  public static Trigger ReverseHopperTrigger;
  public static Trigger enableMoveShoot;
  public static Trigger disableMoveShoot;

  public static Trigger HopperOutake;

  public static Trigger PASS_LOCK;
  public static Trigger HOOD_DROP_LOCK;
  public static Trigger runFFCharcterization;
  public static Trigger runWRCharcterization;


  /**
   * Configures all controls based on the current driver and operator settings.
   * This method should be called during robot initialization.
   */
  public static void configureControls() {
    configureDriverBindings();
    configureSubsystemBindings();
  }

  /** Configure driver joystick axes and driving-related buttons. */
  private static void configureDriverBindings() {
    switch (Constants.driver) {
      case KRITHIK:
        configureDriverCommon();
        break;
      case PROGRAMMERS:
      default:
        configureDriverCommon();
        break;
    }
  }

  /** Shared driver bindings used by all driver profiles for now. */
  private static void configureDriverCommon() {
    // Axes
    DRIVE_FORWARD = () -> applyDriveDeadband(-getY(m_translator));
    DRIVE_STRAFE = () -> applyDriveDeadband(-getX(m_translator));
    DRIVE_ROTATE = () -> applyDriveDeadband(-getTwist(m_rotator)/2);

    // Buttons / modes
    RESET_GYRO = m_translator.button(12);
    DRIVE_HOLD_STOP = m_translator.button(3);
    DRIVE_ROBOT_RELATIVE = m_translator.button(4);

    // POV (hat): 0=up, 90=right, 180=down, 270=left; 45/135/225/315 = diagonals
    POV_UP = new Trigger(() -> m_translator.getHID().getPOV() == 0);
    POV_DOWN = new Trigger(() -> m_translator.getHID().getPOV() == 180);
    POV_LEFT = new Trigger(() -> m_translator.getHID().getPOV() == 270);
    POV_RIGHT = new Trigger(() -> m_translator.getHID().getPOV() == 90);
    POV_UP_RIGHT = new Trigger(() -> m_translator.getHID().getPOV() == 45);
    POV_DOWN_RIGHT = new Trigger(() -> m_translator.getHID().getPOV() == 135);
    POV_DOWN_LEFT = new Trigger(() -> m_translator.getHID().getPOV() == 225);
    POV_UP_LEFT = new Trigger(() -> m_translator.getHID().getPOV() == 315);
  }

  // yushy_boi was here
  /**
   * Configure weapons / subsystem controls (hood, intake, turret, hopper, etc.).
   */
  private static void configureSubsystemBindings() {
    INTAKE_TRIGGER = m_weaponsController.rightTrigger();

    // B button: run flywheel at aimed velocity
    flyWheelTrigger = m_weaponsController.b();
    // Disable old hopper outtake mapping (no button assigned now)
    HopperOutake = new Trigger(() -> false);

    HOOD_DROP_LOCK = m_weaponsController.rightBumper();

    EXTEND_CLIMBER = m_weaponsController.start();
    RETRACT_CLIMBER = m_weaponsController.back();

    EXTEND_INTAKE = m_weaponsController.x();
    RETRACT_INTAKE = m_weaponsController.y();

    PASS_LOCK = m_weaponsController.a();

    // Translator buttons:
    // 5: hopper + kicker
    // 6: intake + hopper
    HopperTrigger = m_weaponsController.leftTrigger();
    ReverseHopperTrigger = m_weaponsController.leftBumper();

    reverseKicker = m_translator.button(10);

    enableMoveShoot = m_rotator.button(1);
    disableMoveShoot = m_rotator.button(2);
    agitateTrigger = m_weaponsController.povUp();





  }
}
