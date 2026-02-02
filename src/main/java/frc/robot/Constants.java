// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.hal.AllianceStationID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Robot mode configuration - can be overridden for testing */
  public static final Mode mode = Mode.REAL;

  /** Current driver configuration */
  public static final Drivers driver = Drivers.KRITHIK;

  /** Current operator configuration */
  public static final Operators operator = Operators.KRITHIK;

  /** Current robot mode (automatically determined based on runtime environment) */
  public static final Mode currentMode = getRobotMode();

  /** Enable tuning mode for PID and other parameters */
  public static final boolean tuning = true;

  /** Enable tuning mode (duplicate - consider removing) */
  public static final boolean tuningMode = true;
  
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Enable vision processing */
  public static final boolean useVision = true;

  /** Robot operation modes */
  public enum Mode {
    /** Running on a real robot */
    REAL,

    /** Running a physics simulator */
    SIM,

    /** Test bot mode */
    TEST,

    /** Replaying from a log file */
    REPLAY
  }

  /** Available driver configurations */
  public enum Drivers {
    KRITHIK,
    ZACH,
    YASHA,
    PROGRAMMERS
  }

  /** Available operator configurations */
  public enum Operators {
    KRITHIK,
    ZACH,
    YASHA,
    PROGRAMMERS
  }

  /**
   * Determines the current robot mode based on runtime environment.
   *
   * @return The current robot mode (REAL, SIM, TEST, or REPLAY)
   */
  public static Mode getRobotMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    }
    if (RobotBase.isSimulation()) {
      switch (mode) {
        case REAL:
          // System.out.println("WARNING: Running in real mode while in simulation");
          // Fall through to SIM mode
        case SIM:
          return Mode.SIM;
        case TEST:
          return Mode.TEST;
        case REPLAY:
          return Mode.REPLAY;
        default:
          return Mode.SIM;
      }
    }
    return Mode.REAL;
  }


  /** Constants for NEO motor specifications */
  public static final class NeoMotorConstants {
    /** Free speed of NEO motor in RPM */
    public static final double kFreeSpeedRpm = 5676;
  }

  /** Constants for operator controls */
  public static class OperatorConstants {
    /** Deadband for joystick inputs to prevent drift */
    public static final double kDriveDeadband = 0.025;

    /** Multiplier for translation (forward/strafe) inputs */
    public static final double translationMultiplier = 1.0;

    /** Multiplier for rotation inputs */
    public static final double rotationMultiplier = 1.0;
  }

  /** Electrical layout - CAN IDs and DIO ports for core robot components */
  public static class ElectricalLayout {
    // Controllers
    /** Driver joystick/controller port */
    public static final int CONTROLLER_DRIVER_ID = 0;

    /** Operator joystick/controller port */
    public static final int CONTROLLER_OPERATOR_ID = 1;

    // LED system
    /** Blinkin LED controller PWM port */
    public static final int BLINKIN_LED_CONTROLLER_PORT = 7;

    // Sample motor (example subsystem)
    /** Sample motor CAN ID */
    public static final int SAMPLE_MOTOR_CAN_ID = 20;

    // Note: Game-specific subsystem CAN IDs should be added here when the 2026 game is announced
  }

  /** Mathematical constant PI */
  public static final double PI = 3.141592653589793238462643;

  /** Control loop update period in seconds */
  public static final double UPDATE_PERIOD = 0.010; // seconds

  /** Current limit for NEO 550 motors in amps */
  public static final int NEO_550_CURRENT_LIMIT = 25;

  /** Current limit for NEO Vortex motors in amps */
  public static final int NEO_VORTEX_CURRENT_LIMIT = 60;

  /** Encoder resolution for quadrature encoders (counts per revolution) */
  public static final int QUADRATURE_COUNTS_PER_REV = 8192;
  // Reference: https://www.revrobotics.com/rev-11-1271/

  /** Current limit for standard NEO motors in amps */
  public static final int NEO_CURRENT_LIMIT = 40;


  /** Constants for vision processing and AprilTag detection */
  public static class VisionConstants {
    // Camera configuration
    /** AprilTag pipeline number */
    public static final double kAprilTagPipeline = 1;

    /** LED off value for camera */
    public static final double kLightOffValue = 0;

    // PID values for driving with vision
    /** Distance tolerance for vision alignment */
    public static final double kDistanceTolerance = 0;

    /** Proportional gain for X-axis vision control */
    public static final double kPX = 0;

    /** Static feedforward for drive control */
    public static final double kSDrive = 0;

    /** Proportional gain for Y-axis vision control */
    public static final double kPY = 0;

    /** General tolerance for vision alignment */
    public static final double kTolerance = 0;

    /** Proportional gain for turn control */
    public static final double kPTurn = 0.0065;

    /** Integral gain for turn control */
    public static final double kITurn = 0;

    /** Derivative gain for turn control */
    public static final double kDTurn = 0.003;

    /** Static feedforward for turn control */
    public static final double kSTurn = 0.025;

    // Camera physical configuration
    /** Limelight camera height from ground in meters */
    public static final double limelightHeight = Units.inchesToMeters(11.5);

    /** Limelight camera angle from horizontal in degrees */
    public static final double limelightAngle = 30.5;

    /** AprilTag width in meters */
    public static final double apriltagWidth = Units.inchesToMeters(6.5);

    /** Tolerance for AprilTag alignment in meters */
    public static final double aprilTagAlignTolerance = 0.5;

    // Note: Game-specific AprilTag heights should be added here when the 2026 game is announced



  }

  public static final class MechanismConstants {
    /** Shooter: Turret base position relative to robot center */
    public static final Transform3d robotToTurret = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(4.75),  // X: forward/back
            Units.inchesToMeters(0),     // Y: left/right
            Units.inchesToMeters(14.25)), // Z: height
        new Rotation3d(0.0, 0.0, -Math.PI / 2.0));

    /** Transform from turret pivot to hood pivot point */
    public static final Transform3d turretToHood = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0.5), // X: forward from turret pivot
            Units.inchesToMeters(4.0), // Y: left/right
            Units.inchesToMeters(4.0)), // Z: height
        new Rotation3d(Units.degreesToRadians(18),0.0, 0.0));

    /** Intake Pivot: Mounted forward of robot center */
    // public static final Transform3d robotToIntakePivot = new Transform3d(
    //     new Translation3d(
    //         Units.inchesToMeters(-10.752), // X: forward/back
    //         Units.inchesToMeters(0.0),  // Y: left/right
    //         Units.inchesToMeters(10.105)), // Z: height
    //     new Rotation3d(-Units.degreesToRadians(IntakeConstants.DEPLOYED_POSITION_degrees),0,-Math.PI/2));

    /** Hopper: Base position for linked floor extension (254-style linkage) */
    public static final Transform3d robotToHopperBase = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(3), // X: forward/back
            Units.inchesToMeters(-0.125), // Y: left/right
            Units.inchesToMeters(16.496)), // Z: height
        new Rotation3d(0,0,-Math.PI/2));
    
    /** Maximum distance the hopper floor extends when intake is fully deployed */
    public static final double kMaxHopperExtensionMeters = Units.inchesToMeters(10.0);

    /** Climb: Telescoping arm base position relative to robot center */
    public static final Transform3d robotToClimbBase = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0), // X: forward/back (negative is back)
            Units.inchesToMeters(0),   // Y: left/right
            Units.inchesToMeters(-8)),
        new Rotation3d(0,0,-Math.PI/2));
  }
}
