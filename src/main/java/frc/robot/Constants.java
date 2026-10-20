// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for robot-wide numerical or boolean constants.
 * Subsystem-specific constants live next to their subsystem (see kickerConstants).
 */
public final class Constants {

  /** Robot mode used when running in simulation (REAL on a real robot is detected automatically) */
  public static final Mode mode = Mode.SIM;

  /** Current robot mode (automatically determined based on runtime environment) */
  public static final Mode currentMode = getRobotMode();

  /** Robot operation modes */
  public enum Mode {
    /** Running on a real robot */
    REAL,

    /** Running a physics simulator */
    SIM,

    /** Replaying from a log file */
    REPLAY
  }

  /**
   * Determines the current robot mode based on runtime environment.
   *
   * @return REAL on a roboRIO, otherwise whatever {@link #mode} is set to (SIM or REPLAY)
   */
  public static Mode getRobotMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    }
    return mode == Mode.REPLAY ? Mode.REPLAY : Mode.SIM;
  }

  /** Controller ports */
  public static final class OperatorConstants {
    /** Xbox controller port in the Driver Station */
    public static final int kControllerPort = 0;
  }
}
