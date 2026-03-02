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
 *  - Driver axes + basic drive buttons: edit {@link #configureDriverBindings()}.
 *  - Weapons / subsystems (hood, intake, shooter, hopper, etc.):
 *      edit {@link #configureSubsystemBindings()}.
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
      return ((CommandJoystick) controller).getHID().getRawAxis(3);
    } else {
      // getRawAxis(2) returns the raw twist/rotation axis value
      return controller.getHID().getRawAxis(2);
    }
  }

  /** Weapons/operator controller for subsystem controls */
  public static final CommandXboxController m_weaponsController =
      new CommandXboxController(Constants.ElectricalLayout.CONTROLLER_WEAPONS_ID);

  // Useful for things that don't need to be triggered
  /** Empty trigger that never fires */
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);

  /** Empty double supplier that always returns 0.0 */
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  private static double applyDriveDeadband(double value) {
    return MathUtil.applyDeadband(value, Constants.OperatorConstants.kDriveDeadband);
  }

  public static Trigger TICK_2_HOOD;
  public static Trigger TICK_37_HOOD;
  public static Trigger MOVE_HOOD;
  public static DoubleSupplier MOVE_HOOD_JOYSTICK;
  public static Trigger EXTEND_CLIMBER;
  public static Trigger RETRACT_CLIMBER;

  // Drive controls
  /** Forward/backward drive input */
  public static DoubleSupplier DRIVE_FORWARD;

  /** Left/right strafe drive input */
  public static DoubleSupplier DRIVE_STRAFE;

  /** Rotation drive input */
  public static DoubleSupplier DRIVE_ROTATE;

  /** Trigger to enable slow mode */
  public static Trigger DRIVE_SLOW;
  public static Trigger EXTEND_INTAKE;

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
  //public static DoubleSupplier INTAKE;

  public static Trigger INTAKE_TRIGGER;
  public static Trigger RETRACT_INTAKE;

  /** Trigger for quasistatic reverse test */
  public static Trigger QUASISTATIC_REVERSE;

  /** Trigger for dynamic forward test */
  public static Trigger DYNAMIC_FORWARD;

  /** Trigger for dynamic reverse test */
  public static Trigger DYNAMIC_REVERSE;

  public static Trigger hoodTrigger;
  public static Trigger reverseHoodTrigger;

  public static Trigger flyWheelTrigger;
  public static Trigger reverseFlyWheelTrigger; 

  public static Trigger turretTrigger0;
  public static Trigger turretTrigger90;
  public static Trigger turretTrigger180;
  public static Trigger turretTrigger45;

  public static Trigger HopperTrigger;
  public static Trigger ReverseHopperTrigger;
  public static Trigger HopperStopTrigger;
  public static Trigger stopFlyWheelTrigger;

  public static Trigger 
  HopperOutake;
  


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
    DRIVE_ROTATE = () -> applyDriveDeadband(-getTwist(m_translator));

    // Buttons / modes
    RESET_GYRO = m_translator.button(12);
    DRIVE_SLOW = m_translator.button(1);
    DRIVE_STOP = m_translator.button(2);
    DRIVE_HOLD_STOP = m_translator.button(3);
    DRIVE_ROBOT_RELATIVE = m_translator.button(4);
  }
  // yushy_boi was here
  /** Configure weapons / subsystem controls (hood, intake, turret, hopper, etc.). */
  private static void configureSubsystemBindings() {
    hoodTrigger = m_weaponsController.leftTrigger();
    reverseHoodTrigger = m_weaponsController.leftBumper();

    INTAKE_TRIGGER = m_weaponsController.rightTrigger();

    flyWheelTrigger = m_weaponsController.a();
    reverseFlyWheelTrigger = m_weaponsController.b();
    stopFlyWheelTrigger = m_translator.button(11);

    turretTrigger0 = m_weaponsController.povUp();
    turretTrigger90 = m_weaponsController.povDown();
    turretTrigger180 = m_weaponsController.povLeft();
    turretTrigger45 = m_weaponsController.povRight();

    EXTEND_INTAKE = m_weaponsController.x();
    RETRACT_INTAKE = m_weaponsController.y();

    MOVE_HOOD = m_weaponsController.rightBumper();
    MOVE_HOOD_JOYSTICK = () -> m_weaponsController.getRightY();

    TICK_2_HOOD = m_translator.button(8);
    TICK_37_HOOD = m_translator.button(7);

    HopperTrigger = m_translator.button(6);
    ReverseHopperTrigger = m_translator.button(5);
    HopperStopTrigger = m_translator.button(10);
    
    HopperOutake = m_translator.button(12);
  }
}
