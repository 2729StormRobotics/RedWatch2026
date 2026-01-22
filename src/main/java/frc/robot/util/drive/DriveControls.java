package frc.robot.util.drive;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Centralized control configuration for driver and operator inputs for FRC 2026.
 * Maps physical controller inputs to logical control actions for the 17-motor Project Titan bot.
 */
public class DriveControls {
  // --- Controllers ---
  
  /** Translator joystick for forward/strafe movement (Port 0) */
  public static final CommandGenericHID m_translator;

  /** Rotator joystick for rotation control (Port 1) */
  public static final CommandGenericHID m_rotator;

  /** Weapons/operator controller for subsystem controls (Port 2) */
  public static final CommandXboxController m_operator = new CommandXboxController(2);

  // Static initializer to set up controllers based on mode
  static {
    if (Constants.currentMode == Constants.Mode.REAL) {
      m_translator = new CommandJoystick(0);
      m_rotator = new CommandJoystick(1);
    } else {
      m_translator = new CommandGenericHID(0);
      m_rotator = new CommandGenericHID(1);
    }
  }

  // --- Helper Methods ---

  private static double getY(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getY();
    } else {
      return -controller.getHID().getRawAxis(1);
    }
  }

  private static double getX(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getX();
    } else {
      return -controller.getHID().getRawAxis(0);
    }
  }

  private static double getTwist(CommandGenericHID controller) {
    if (controller instanceof CommandJoystick) {
      return ((CommandJoystick) controller).getTwist();
    } else {
      return -controller.getHID().getRawAxis(2);
    }
  }

  // --- Logical Triggers & Suppliers ---

  // Drive Outputs
  public static DoubleSupplier DRIVE_FORWARD;
  public static DoubleSupplier DRIVE_STRAFE;
  public static DoubleSupplier DRIVE_ROTATE;

  // Drive Settings
  public static Trigger DRIVE_SLOW;
  public static Trigger RESET_GYRO;
  public static Trigger FIELD_RELATIVE_TOGGLE;
  public static Trigger DRIFT_BRACE; // "The Brick" mode

  // Shooter & Scoring (The "Insane" Commands)
  public static Trigger AUTO_SCORE;      // Coordinates Turret/Flywheel/Hood + Kicker
  public static Trigger MOVE_AND_SHOOT;  // Toggles vector compensation
  public static Trigger MANUAL_SHOOT;    // Force Kicker
  public static Trigger RESET_TURRET;    // Re-run CRT logic

  // Intake & Hopper
  public static Trigger INTAKE_COLLECT;  // IntelligentCollection (Touch it, Own it)
  public static Trigger INTAKE_EJECT;    // EmergencyEject
  public static Trigger HOPPER_AGITATE;  // Manual "Pulse-and-Shake"

  // Climb
  public static Trigger CLIMB_SEQUENCE;

  /**
   * Configures all controls based on the current driver and operator settings.
   */
  public static void configureControls() {
    // --- Operator Controls (Common to all drivers) ---
    AUTO_SCORE = m_operator.rightTrigger();
    INTAKE_COLLECT = m_operator.leftTrigger();
    INTAKE_EJECT = m_operator.b();
    HOPPER_AGITATE = m_operator.x();
    CLIMB_SEQUENCE = m_operator.start();
    MANUAL_SHOOT = m_operator.rightBumper();
    MOVE_AND_SHOOT = m_operator.y();

    // --- Driver Specific Configurations ---
    switch (Constants.driver) {
      case KRITHIK:
        // Movement mapped to the Rotator stick
        DRIVE_FORWARD = () -> (-getY(m_rotator));
        DRIVE_STRAFE = () -> (-getX(m_rotator));
        DRIVE_ROTATE = () -> (getTwist(m_rotator) * 0.5); // Half-speed rotation sensitivity
        
        RESET_GYRO = m_rotator.button(12);
        DRIFT_BRACE = m_rotator.button(1); // Thumb button for "Brick" mode
        
        // Settings on Translator stick
        DRIVE_SLOW = m_translator.button(1);
        FIELD_RELATIVE_TOGGLE = m_translator.button(2);
        RESET_TURRET = m_translator.button(7);
        break;

      case PROGRAMMERS:
      default:
        // Standard Tank/Swerve split
        DRIVE_FORWARD = () -> (-getY(m_translator));
        DRIVE_STRAFE = () -> (-getX(m_translator));
        DRIVE_ROTATE = () -> (-getTwist(m_rotator));
        
        RESET_GYRO = m_translator.button(12);
        DRIVE_SLOW = m_translator.button(1);
        FIELD_RELATIVE_TOGGLE = m_translator.button(2);
        DRIFT_BRACE = m_translator.button(3);
        RESET_TURRET = m_operator.back();
        break;
    }
  }
}