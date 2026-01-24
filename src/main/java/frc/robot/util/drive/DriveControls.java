package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Centralized control configuration for driver and operator inputs for FRC 2026.
 * Maps physical controller inputs to logical control actions for the 17-motor Project Titan bot.
 * Updated to support both physical Xbox/Joysticks and Simulation GUI buttons.
 */
public class DriveControls {
  // --- Controllers ---
  
  /** Translator joystick for forward/strafe movement (Port 0) */
  public static final CommandGenericHID m_translator;

  /** Rotator joystick for rotation control (Port 1) */
  public static final CommandGenericHID m_rotator;

  /** Weapons/operator controller (Port 2) - Standard Xbox layout in Real, Generic in Sim */
  public static final CommandGenericHID m_operator;

  // Static initializer to set up controllers based on mode
  static {
    if (Constants.currentMode == Constants.Mode.REAL) {
      m_translator = new CommandJoystick(0);
      m_rotator = new CommandJoystick(1);
      m_operator = new CommandXboxController(2);
    } else {
      // In SIM, use GenericHID for all to ensure the Sim GUI "Buttons" work reliably
      m_translator = new CommandGenericHID(1);
      m_rotator = new CommandGenericHID(0);
      m_operator = new CommandGenericHID(2);
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
  public static Trigger DRIFT_BRACE; 

  // Shooter & Scoring
  public static Trigger AUTO_SCORE;      
  public static Trigger MOVE_AND_SHOOT;  
  public static Trigger MANUAL_SHOOT;    
  public static Trigger RESET_TURRET;    

  // Intake & Hopper
  public static Trigger INTAKE_COLLECT;  
  public static Trigger INTAKE_EJECT;    
  public static Trigger HOPPER_AGITATE;  

  // Climb
  public static Trigger CLIMB_SEQUENCE;

  /**
   * Configures all controls. 
   * Handles the abstraction between Real (Xbox axis/buttons) and Sim (Generic GUI buttons).
   */
  public static void configureControls() {
    // --- Operator Mapping ---
    if (Constants.currentMode == Constants.Mode.REAL && m_operator instanceof CommandXboxController xbox) {
      // High-fidelity mapping for the actual Xbox Controller
      AUTO_SCORE = xbox.rightTrigger();
      INTAKE_COLLECT = xbox.leftTrigger();
      INTAKE_EJECT = xbox.b();
      HOPPER_AGITATE = xbox.x();
      CLIMB_SEQUENCE = xbox.start();
      MANUAL_SHOOT = xbox.rightBumper();
      MOVE_AND_SHOOT = xbox.y();
    } else {
      // Simulation Mapping: Using specific button IDs makes it easy to click in the Sim GUI
      // Buttons 1-10 on Joystick Port 2
      AUTO_SCORE = m_operator.button(1);      // Trigger Button 1
      INTAKE_COLLECT = m_operator.button(2);  // Trigger Button 2
      INTAKE_EJECT = m_operator.button(3);
      HOPPER_AGITATE = m_operator.button(4);
      MANUAL_SHOOT = m_operator.button(5);
      MOVE_AND_SHOOT = m_operator.button(6);
      RESET_TURRET = m_operator.button(7);
      CLIMB_SEQUENCE = m_operator.button(8);
    }

    // --- Driver Specific Configurations ---
    switch (Constants.driver) {
      case KRITHIK:
        DRIVE_FORWARD = () -> (-getY(m_rotator));
        DRIVE_STRAFE = () -> (-getX(m_rotator));
        DRIVE_ROTATE = () -> (getTwist(m_rotator) * 0.5);
        
        RESET_GYRO = m_rotator.button(12);
        DRIFT_BRACE = m_rotator.button(1); 
        
        DRIVE_SLOW = m_translator.button(1);
        FIELD_RELATIVE_TOGGLE = m_translator.button(2);
        
        // In simulation, ensure the reset button is easy to find
        if (Constants.currentMode == Constants.Mode.SIM) {
            RESET_TURRET = m_translator.button(10);
        } else {
            RESET_TURRET = m_translator.button(7);
        }
        break;

      case PROGRAMMERS:
      default:
        DRIVE_FORWARD = () -> (-getY(m_translator));
        DRIVE_STRAFE = () -> (-getX(m_translator));
        DRIVE_ROTATE = () -> (-getTwist(m_rotator));
        
        RESET_GYRO = m_translator.button(12);
        DRIVE_SLOW = m_translator.button(1);
        FIELD_RELATIVE_TOGGLE = m_translator.button(2);
        DRIFT_BRACE = m_translator.button(3);
        
        if (Constants.currentMode != Constants.Mode.SIM && m_operator instanceof CommandXboxController xbox) {
             RESET_TURRET = xbox.back();
        } else {
             RESET_TURRET = m_operator.button(9);
        }
        break;
    }
  }
}