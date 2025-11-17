package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class DriveControls {
  // Controllers
  public static final CommandJoystick m_translator = new CommandJoystick(0) ;
  public static final CommandJoystick m_rotator = new CommandJoystick(1);
  public static final CommandXboxController m_weaponsController = new CommandXboxController(0);

  // Useful for things that don't need to be triggered
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  // Drive controls
  public static DoubleSupplier DRIVE_FORWARD;
  public static DoubleSupplier ELEVATOR_JOYSTICK;
  public static DoubleSupplier DRIVE_STRAFE;
  public static DoubleSupplier DRIVE_ROTATE;
  public static Trigger DRIVE_SLOW;
  public static Trigger DRIVE_STOP;
  public static Trigger DRIVE_HOLD_STOP;

  // drive modes
  public static Trigger DRIVE_ROBOT_RELATIVE;
  public static Trigger DRIVE_FIELD_RELATIVE;
  public static Trigger DRIVE_SPEAKER_AIM;

  // Drive Angle Locks
  public static Trigger LOCK_BACK;
  public static Trigger LOCK_PICKUP;
  public static Trigger LOCK_PASS;
  public static Trigger LOCK_ON_AMP;

  // Drive Trajectories
  public static Trigger DRIVE_AMP;

  // Pivot Controls
  public static DoubleSupplier PIVOT_ROTATE;
  public static Trigger PIVOT_AMP;
  public static Trigger PIVOT_ZERO;
  public static Trigger PIVOT_TO_SPEAKER;
  public static Trigger PIVOT_PODIUM;
  public static Trigger PIVOT_ANYWHERE;
  public static Trigger PIVOT_HOLD;
  public static Trigger PIVOT_AND_REV;

  // Intake Controls
  public static Trigger INTAKE_IN;
  public static Trigger INTAKE_OUT;
  public static Trigger INTAKE_THEN_LOAD;

  // Ground Intake
  public static Trigger GROUND_INTAKE_IN;
  public static Trigger GROUND_INTAKE_OUT;

  public static Trigger INTAKE_UNTIL_INTAKED;

  // Shooter Controls
  public static DoubleSupplier SHOOTER_SPEED;
  public static Trigger SHOOTER_FULL_SEND_INTAKE;
  public static Trigger SHOOTER_FULL_SEND;
  public static Trigger SHOOTER_UNJAM;
  public static Trigger SHOOTER_PREPARE_THEN_SHOOT;

  public static Trigger RESET_GYRO;
  // SYSID Controls
  public static Trigger QUASISTATIC_FORWARD;
  public static Trigger QUASISTATIC_REVERSE;
  public static Trigger DYNAMIC_FORWARD;
  public static Trigger DYNAMIC_REVERSE;

  public static Trigger ELEVATOR_L1;
  public static Trigger ELEVATOR_L2;
  public static Trigger ELEVATOR_L3;
  public static Trigger ELEVATOR_L4;
  public static Trigger ELEVATOR_INTAKE;

  // Setup the controls
  public static void configureControls() {
    switch (Constants.driver) {
      case KRITHIK:
        ELEVATOR_JOYSTICK = () -> (-m_weaponsController.getLeftY()/2);
        // Driver controls
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX() );
        DRIVE_ROTATE = () -> (m_rotator.getTwist()/2); // Add negative to make turning inverted 
        RESET_GYRO = m_translator.button(12);
        // Driver Settings
        DRIVE_SLOW = m_translator.button(1); // TBA
        DRIVE_STOP = m_translator.button(2); // TBA
        DRIVE_HOLD_STOP = m_translator.button(3); // TBA

        // Driver Modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4); // TBA
        DRIVE_SPEAKER_AIM = m_translator.button(1); // uses vision

        // ALL BELOW TBD
        // Driver Angle Locks
        LOCK_BACK = m_translator.button(1);
        LOCK_PICKUP = m_translator.button(1);
        LOCK_ON_AMP = m_translator.button(1);
        LOCK_PASS = m_translator.button(1); // uses vision

        DRIVE_AMP = EMPTY_TRIGGER; // uses vision
        break;
      case PROGRAMMERS:
      default:
        // Driver controls
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX());
        DRIVE_ROTATE = () -> (-m_translator.getTwist());
        RESET_GYRO = m_translator.button(12);

        // Driver Settings
        DRIVE_SLOW = m_translator.button(1); // TBA
        DRIVE_STOP = m_translator.button(2); // TBA
        DRIVE_HOLD_STOP = m_translator.button(3); // TBA

        // Driver Modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4); // TBA
        // DRIVE_SPEAKER_AIM = m_translator.leftBumper(); // uses vision

        // ALL BELOW TBD
        // Driver Angle Locks
        LOCK_BACK = m_translator.button(1);
        LOCK_PICKUP = m_translator.button(1);
        LOCK_ON_AMP = m_translator.button(1);
        LOCK_PASS = m_translator.button(1); // uses vision

        DRIVE_AMP = EMPTY_TRIGGER; // uses vision
    }

    switch (Constants.operator) {
      case KRITHIK:
        PIVOT_ROTATE =
            () ->
                (m_weaponsController.getRightTriggerAxis()
                    - m_weaponsController.getLeftTriggerAxis());
        // all tbd
        // Pivot things
        PIVOT_AMP = m_weaponsController.b();
        PIVOT_ZERO = m_weaponsController.a();
        PIVOT_TO_SPEAKER = m_weaponsController.y();
        PIVOT_PODIUM = m_weaponsController.y();
        PIVOT_ANYWHERE = m_weaponsController.button(1); // uses vision
        PIVOT_HOLD = m_weaponsController.start();
        PIVOT_AND_REV = new Trigger(() -> (m_weaponsController.getRightTriggerAxis() > 0.5));
        // intaking things
        INTAKE_IN = m_weaponsController.rightBumper();
        INTAKE_OUT = m_weaponsController.leftBumper();
        INTAKE_UNTIL_INTAKED = EMPTY_TRIGGER;
        INTAKE_THEN_LOAD = m_weaponsController.x();
        // ground intake things
        GROUND_INTAKE_IN = m_weaponsController.rightBumper();
        GROUND_INTAKE_OUT = m_weaponsController.leftBumper();

        // Shooter things
        SHOOTER_SPEED = () -> m_weaponsController.getRightX();
        SHOOTER_FULL_SEND_INTAKE = m_weaponsController.leftStick();
        SHOOTER_FULL_SEND = m_translator.button(1);
        SHOOTER_UNJAM = m_weaponsController.button(1);
        SHOOTER_PREPARE_THEN_SHOOT = m_weaponsController.back();
        break;
      case PROGRAMMERS:
      default:
        // Operator controls
        PIVOT_ROTATE =
            () ->
                (m_weaponsController.getRightTriggerAxis()
                    - m_weaponsController.getLeftTriggerAxis());

        // ALL TBD

        // isn't reading m_weaponsController.getLeftTriggerAxis, must be an issue with the encoder
        PIVOT_AMP = m_weaponsController.button(1);
        PIVOT_ZERO = m_weaponsController.button(2);
        PIVOT_TO_SPEAKER = EMPTY_TRIGGER;
        PIVOT_PODIUM = m_weaponsController.button(1);

        INTAKE_IN = m_weaponsController.rightBumper();
        INTAKE_OUT = m_weaponsController.leftBumper();

        GROUND_INTAKE_IN = m_weaponsController.rightBumper();
        GROUND_INTAKE_OUT = m_weaponsController.leftBumper();

        // SHOOTER_SPEED = m_weaponsController::getRight;

        SHOOTER_FULL_SEND_INTAKE = EMPTY_TRIGGER;
        SHOOTER_FULL_SEND = EMPTY_TRIGGER;
        SHOOTER_UNJAM = EMPTY_TRIGGER;
        SHOOTER_PREPARE_THEN_SHOOT = EMPTY_TRIGGER;
        break;

        // bottom right Left joystick to intake
    }
  }
}
