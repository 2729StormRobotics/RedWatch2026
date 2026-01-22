package frc.robot.subsystems.hopper;

import static frc.robot.Constants.ElectricalLayout.*;

public final class HopperConstants {
  public static final int MOTOR_ID = HOPPER_MOTOR_ID;
  public static final int BEAM_BREAK_PORT = HOPPER_BEAM_BREAK_PORT;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean MOTOR_INVERTED = false;
  public static final double FULL_CURRENT_THRESHOLD = 15.0; // Amps - indicates fullness
  public static final double JAM_CURRENT_THRESHOLD = 20.0; // Amps - indicates jam
  public static final double PULSE_VOLTAGE = 6.0; // Volts for pulse-and-shake
  public static final double MOI_KG_M2 = 0.01;
  
  private HopperConstants() {}
}
