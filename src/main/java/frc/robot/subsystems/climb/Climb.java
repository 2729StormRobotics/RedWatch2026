package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AlphaMechanism3d;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    AlphaMechanism3d.getInstance().setClimb(inputs.positionMeters);
  }

  /**
   * Returns a command that runs the motor at the specified voltage.
   */
  public Command runVoltageCommand(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command runClimb(double percent) {
    return this.run(() -> io.setPercent(percent));
  }

  /**
   * Returns a command that stops the climber motor.
   */
  public Command stopCommand() {
    return this.runOnce(io::stop);
  }

  public Command climbCommand() {
    return runClimb(1);
  }

  public Command retractCommand() {
    return runClimb(-1);
  }
  

  // AUTO COMMANDS
  public Command AutoCommandRaiseClimber(){
    return new SequentialCommandGroup(climbCommand(), new WaitCommand(ClimbConstants.TimeToGoDown), stopCommand());
  }


  public Command AutoCommandPullClimber(){
    return new SequentialCommandGroup(retractCommand(), new WaitCommand(ClimbConstants.TimeToGoDown), stopCommand());
  }
}