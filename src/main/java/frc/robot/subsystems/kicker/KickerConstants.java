package frc.robot.subsystems.kicker;

import static frc.robot.Constants.ElectricalLayout.*;

public final class KickerConstants {
  public static final int MOTOR_ID = KICKER_MOTOR_ID;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean MOTOR_INVERTED = false;
  public static final double FIRE_VOLTAGE = 12.0;
  public static final double MOI_KG_M2 = 0.005;
  
  private KickerConstants() {}
}
