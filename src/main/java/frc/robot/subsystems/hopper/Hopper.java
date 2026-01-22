package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  
  private double desiredVoltage = 0.0;
  private boolean isFull = false;
  private boolean isJammed = false;
  private double lastPulseTime = 0.0;
  private static final double PULSE_DURATION = 0.1; // seconds

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    
    // Fullness detection via stator current
    isFull = inputs.currentAmps > HopperConstants.FULL_CURRENT_THRESHOLD || inputs.beamBreakTriggered;
    
    // Jam detection
    isJammed = inputs.currentAmps > HopperConstants.JAM_CURRENT_THRESHOLD;
    
    // Anti-jam "Pulse-and-Shake" logic
    if (isJammed) {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime - lastPulseTime > PULSE_DURATION * 2) {
        // Pulse reverse
        io.setVoltage(-HopperConstants.PULSE_VOLTAGE);
        lastPulseTime = currentTime;
      } else if (currentTime - lastPulseTime > PULSE_DURATION) {
        // Return to forward
        io.setVoltage(desiredVoltage);
      }
    } else {
      // Normal operation
      io.setVoltage(desiredVoltage);
    }
  }

  public void setVoltage(double volts) {
    desiredVoltage = volts;
  }

  public void stop() {
    desiredVoltage = 0.0;
    io.stop();
  }

  @AutoLogOutput(key = "Hopper/IsFull")
  public boolean isFull() {
    return isFull;
  }

  @AutoLogOutput(key = "Hopper/IsJammed")
  public boolean isJammed() {
    return isJammed;
  }

  public Command runCommand(double volts) {
    return Commands.run(() -> setVoltage(volts), this);
  }
}
