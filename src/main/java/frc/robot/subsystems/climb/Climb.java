package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  // Mechanism visualization
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3.0, 3.0);
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d climbArm;

  public Climb(ClimbIO io) {
    this.io = io;
    
    // Set up mechanism visualization
    // Root at bottom-center of canvas
    root = mechanism.getRoot("ClimbRoot", 1.5, 2.5);
    // Climb arm that extends vertically upward (90° = up)
    climbArm = root.append(
        new LoggedMechanismLigament2d(
            "ClimbArm",
            0.3, // Initial length (retracted)
            90.0, // Angle (pointing up)
            6.0, // Width
            new Color8Bit(Color.kGreen)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    // Update mechanism visualization
    // Convert position (meters) to mechanism length
    // Base length of 0.3 + extension (scaled for visualization)
    double baseLength = 0.3;
    double extensionLength = inputs.positionMeters * 2.0; // Scale factor for visualization
    climbArm.setLength(baseLength + extensionLength);
    
    // Log mechanism
    Logger.recordOutput("Climb/Mechanism", mechanism);
  }

  public void climb() {
    io.setLock(false);
    io.setVoltage(ClimbConstants.CLIMB_VOLTAGE);
  }

  public void retract() {
    io.setVoltage(-ClimbConstants.CLIMB_VOLTAGE);
  }

  public void lock() {
    io.setLock(true);
    io.stop();
  }

  public void stop() {
    io.stop();
  }

  public Command climbCommand() {
    return Commands.runOnce(() -> climb(), this);
  }
}
