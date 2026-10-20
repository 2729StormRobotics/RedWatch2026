package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.kicker.kicker;
import frc.robot.subsystems.kicker.kickerIO;
import frc.robot.subsystems.kicker.kickerIOReal;
import frc.robot.subsystems.kicker.kickerIOSim;

/**
 * This class is where the robot is declared: subsystems, controllers, and button bindings. Very
 * little robot logic lives in {@link Robot}; instead the structure of the robot is declared here.
 */
public class RobotContainer {
  // Subsystems
  private final kicker kicker;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(Constants.OperatorConstants.kControllerPort);

  /**
   * Creates the subsystems with the right IO implementation for the current mode (REAL, SIM, or
   * REPLAY), then binds the controller buttons to commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot: talk to real motor controllers
        kicker = new kicker(new kickerIOReal());
        break;

      case SIM:
        // Simulator: use the physics model
        kicker = new kicker(new kickerIOSim());
        break;

      default:
        // Replay: IO does nothing, inputs come from the log
        kicker = new kicker(new kickerIO() {});
        break;
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: connect buttons to commands. A "binding" says: when THIS happens on the controller,
    // run THAT command.
    //
    //   controller.rightTrigger().whileTrue(...)
    //     -> runs the command while the trigger is held, cancels it when released.
    //        Bind the kicker's runContinuous() command here.
    //
    //   controller.leftTrigger().whileTrue(...)
    //     -> bind the kicker's reverse() command here to clear jams.
    //
    // Other triggers you can use: onTrue (once, on press), onFalse (once, on release),
    // toggleOnTrue (press to start, press again to stop).
  }
}
