package frc.robot.subsystems.climb;

import static frc.robot.Constants.ElectricalLayout.*;

public final class ClimbConstants {
  public static final int LEADER_ID = 15;
  // public static final int FOLLOWER_ID = 18;
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean LEADER_INVERTED = false;
  public static final boolean FOLLOWER_INVERTED = true;
  public static final double CLIMB_VOLTAGE = 10.0;
  public static final double MOI_KG_M2 = 0.01;
  
  private ClimbConstants() {}
}