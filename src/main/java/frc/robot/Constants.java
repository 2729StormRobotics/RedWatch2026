// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.awt.geom.Point2D;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode mode = Mode.REAL;
  public static final Drivers driver = Drivers.KRITHIK;
  public static final Operators operator = Operators.KRITHIK;

  public static final Mode currentMode = getRobotMode();

  // public static final Mode currentMode = Mode.SIM;

  public static final boolean tuning = true;

  public static final boolean tuningMode = true;
  public static final boolean useVision = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /* Test bot */
    TEST,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Drivers {
    DAN,
    KRITHIK,
    AMOGH,
    NITHILAN,
    ASHLEIGH,
    YASHA,
    PROGRAMMERS
  }

  public static enum Operators {
    KRITHIK,
    AMOGH,
    NITHILAN,
    ASHLEIGH,
    YASHA,
    PROGRAMMERS
  }

  public static Mode getRobotMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    }
    if (RobotBase.isSimulation()) {
      switch (mode) {
        case REAL:
          // System.out.println("WARNING: Running in real mode while in simulation");
        case SIM:
          return Mode.SIM;
        case TEST:
          return Mode.TEST;
        case REPLAY:
          return Mode.REPLAY;
      }
    }
    return Mode.REAL;
  }

  public static final Point2D[] ShootingPoints =
      new Point2D[] { // array of exp determined data points of (dist, angle)
        new Point2D.Double(-1, 47.5),
        new Point2D.Double(0.94, 47.5),
        new Point2D.Double(1.247, 44.5),
        new Point2D.Double(1.507, 40.3),
        new Point2D.Double(1.75, 35.8),
        new Point2D.Double(1.9, 34.5),
        new Point2D.Double(2.3, 30.2),
        new Point2D.Double(2.495, 26.3),
        new Point2D.Double(2.755, 24.9),
        new Point2D.Double(3, 23.2),
        new Point2D.Double(3.395, 21.85),
        new Point2D.Double(3.75, 20),
        new Point2D.Double(4.03, 19.6)
      };

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class OperatorConstants {

    // joystick settings
    public static final double kDriveDeadband = 0.025;
    public static final double translationMultiplier = 1;
    public static final double rotationMultiplier = 1;
  }

  public static class ElectricalLayout {
    // Controllers
    public static final int CONTROLLER_DRIVER_ID = 0;
    public static final int CONTROLLER_OPERATOR_ID = 1;

    // PLACEHOLDER Intake
    public static final int GROUND_INTAKE_MOTOR = 13; // 13
    // Indexer Motor
    public static final int INDEXER_MOTOR = 12;
    // Intake Sensors
    public static final int INTAKE_PHOTO_ELECTRIC = 0;

    public static final int PIVOT_ARM_ID = 9;

    public static final int ABSOLUTE_ENCODER_ID = 8;

    // LED
    public static final int BLINKIN_LED_CONTROLLER_PORT = 7;

    // Hanger Motors
    public static final int HANGER_MOTOR_LEFT = 16;
    public static final int HANGER_MOTOR_RIGHT = 15;

    // Shooter
    public static final int SHOOTER_LEFT_ID = 14; // master
    public static final int SHOOTER_RIGHT_ID = 10;
    public static final int PHOTOELECTRIC_SENSOR_CHANNEL = 9; // NEEDS TO BE CHANGED
  }
  ;

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public static final int NEO_550_CURRENT_LIMIT = 25; // amps
  public static final int NEO_VORTEX_CURRENT_LIMIT = 60;
  public static final int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  public static final int NEO_CURRENT_LIMIT = 40; // amps

  // {distance, rpm, angle} The distance column must go from lowest to highest, top to bottom
  // manually insert velocity!!!!!
  public static final double[][] LookupTable = {
    {0, 0, 38},
    {0.1151611524, 0, 38},
    {0.3522449123, 0, 38},
    {0.8765935905, 0, 38},
    {1.46, 0, 42},
    {1.7, 0, 38},
    {1.959336833, 0, 50.81632738},
    {2.823481946, 0, 55},
    {3.211524819, 0, 56},
    {4.258293028, 0, 57},
    {5, 0, 60}
  };

  public static class VisionConstants {
    // Camera configuration
    public static final double kAprilTagPipeline = 1;
    public static final double kLightOffValue = 0;

    // PID values for driving with vision
    public static final double kDistanceTolerance = 0;
    public static final double kPX = 0;
    public static final double kSDrive = 0;
    public static final double kPY = 0;
    public static final double kTolerance = 0;
    public static final double kPTurn = 0.0065; // 0.008
    public static final double kITurn = 0;
    public static final double kDTurn = 0.003; // 0.001
    public static final double kSTurn = .025;

    // Heights for detecting distance away from apriltag
    public static final double limelightHeight = Units.inchesToMeters(11.5);
    public static final double limelightAngle = 30.5; // degrees
    public static final double apriltagWidth = Units.inchesToMeters(6.5);
    public static final double speakerTagHeight =
        Units.inchesToMeters(54) + Units.inchesToMeters(apriltagWidth / 2);
    public static final double ampTagHeight =
        Units.inchesToMeters(53.875) + Units.inchesToMeters(apriltagWidth / 2);
    public static final double stageTagHeight =
        Units.inchesToMeters(53.875) + Units.inchesToMeters(apriltagWidth / 2);
    public static final double aprilTagAlignTolerance = 0.5;

    public static final double kNoteTolerance = 2.0;
    public static final double kPNoteTurn = 0.008;
  }
}
