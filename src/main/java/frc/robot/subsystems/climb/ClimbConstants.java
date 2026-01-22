package frc.robot.subsystems.climb;

import static frc.robot.Constants.ElectricalLayout.*;

public final class ClimbConstants {
  public static final int MOTOR_ID = CLIMB_MOTOR_ID;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean MOTOR_INVERTED = false;
  public static final double CLIMB_VOLTAGE = 10.0;
  public static final double MOI_KG_M2 = 0.01;
  
  private ClimbConstants() {}
}
