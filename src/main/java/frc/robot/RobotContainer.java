// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;
import java.lang.constant.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.drive.DriveControls;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOReal;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
// import frc.robot.subsystems.climb.Climb;
// import frc.robot.subsystems.climb.ClimbIO;
// import frc.robot.subsystems.climb.ClimbIOReal;
// import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.kicker.kicker;
import frc.robot.subsystems.kicker.kickerConstants;
import frc.robot.subsystems.kicker.kickerIO;
import frc.robot.subsystems.kicker.kickerIOReal;
import frc.robot.subsystems.kicker.kickerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperConstants;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final kicker kicker;
  private final Intake intake;
  private final Hopper hopper;

  // LEDs
  private final BlinkinLEDController ledController = BlinkinLEDController.getInstance();

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d field;

  private SwerveDriveSimulation driveSimulation = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * Initializes subsystems based on the current robot mode (REAL, SIM, or REPLAY).
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        HoodIOReal hoodIOReal = new HoodIOReal();
        shooter = new Shooter(
            new FlywheelIOReal(),
            hoodIOReal,
            new TurretIOReal(hoodIOReal.motor),
            drive,
            driveSimulation);
        kicker = new kicker(new kickerIOReal());
        intake = new Intake(new IntakeIOReal());
        hopper = new Hopper(new HopperIOReal());

        // Vision subsystem with real Limelight cameras
        vision =
            new Vision(
                new VisionIOLimelight(VisionConstants.LEFT_LIMELIGHT_NAME),
                new VisionIOLimelight(VisionConstants.RIGHT_LIMELIGHT_NAME),
                drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        shooter = new Shooter(
            new FlywheelIOSim(),
            new HoodIOSim(),
            new TurretIOSim(),
            drive,
            driveSimulation);
        kicker = new kicker(new kickerIOSim());
        intake = new Intake(new IntakeIOSim(driveSimulation));
        hopper = new Hopper(new HopperIOSim());
        
        // Vision subsystem with simulation IO (no vision data)
        vision =
            new Vision(
                new VisionIOSim(),
                new VisionIOSim(),
                drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        HoodIOReal hoodIORealReplay = new HoodIOReal();
        shooter = new Shooter(
            new FlywheelIOReal(),
            hoodIORealReplay,
            new TurretIOReal(hoodIORealReplay.motor),
            drive,
            driveSimulation);
        kicker = new kicker(new kickerIOReal());
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIOReal());


        // Vision subsystem with empty IO for replay
        vision =
            new Vision(
                new VisionIO() {},
                new VisionIO() {},
                drive);
        break;
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    System.out.println("[Init] Setting up Path Planner Logging");

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          field.setRobotPose(pose);
          Logger.recordOutput("PathPlanner/RobotPose", pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          field.getObject("target pose").setPose(pose);
          Logger.recordOutput("PathPlanner/TargetPose", pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          field.getObject("path").setPoses(poses);
          Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
        });

    // Set up auto routines chooser
    System.out.println("[Init] Setting up Logged Auto Chooser");
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines for drive characterization
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Resets the gyro yaw angle to zero.
   * Useful for resetting the robot's heading reference.
   */
  public void reset() {
    // drive.();
  }

  private void configureButtonBindings() {
    // Configure drive controls based on driver preferences
    DriveControls.configureControls();

    // Set LED to orange on initialization
    ledController.orange();


    // Shooter / Kicker controls
    // TICK_2_HOOD.onTrue(shooter.runPositionCommand(2).andThen(shooter.stopCommand()));
    // TICK_37_HOOD.onTrue(shooter.runPositionCommand(37).andThen(shooter.stopCommand()));
    // MOVE_HOOD.onTrue(shooter.runPositionCommand(19 + (MOVE_HOOD_JOYSTICK * 18)));
    // shooter.setDefaultCommand(shooter.runPositionCommandConstant(DriveControls.m_weaponsController));

    // flyWheelTrigger.whileTrue(
    //     Commands.parallel(
    //         Commands.run(() -> shooter.setFlywheelVelocity(80), shooter),
    //         Commands.run(() -> kicker.setVoltage(1), kicker)));
    // flyWheelTrigger.onFalse(
    //     Commands.parallel(
    //         Commands.runOnce(shooter::stop, shooter),
    //         Commands.runOnce(kicker::stop, kicker)));

    // reverseFlyWheelTrigger.whileTrue(
    //     Commands.parallel(
    //         Commands.run(() -> shooter.setFlywheelVelocity(-40), shooter),
    //         Commands.run(() -> kicker.setVoltage(-0.5), kicker)));
    // reverseFlyWheelTrigger.onFalse(
    //     Commands.parallel(
    //         Commands.runOnce(shooter::stop, shooter),
    //         Commands.runOnce(kicker::stop, kicker)));

    // turretTrigger0.onTrue(Commands.runOnce(() -> shooter.setTurretAngle(0)));
    // turretTrigger180.onTrue(Commands.runOnce(() -> shooter.setTurretAngle(Math.PI)));
    // turretTriggerNegative90.onTrue(Commands.runOnce(() -> shooter.setTurretAngle((3 * Math.PI) / 2)));
    // turretTrigger90.onTrue(Commands.runOnce(() -> shooter.setTurretAngle(Math.PI / 2)));

    // Climb Controls
    // EXTEND_CLIMBER.whileTrue(climb.climbCommand());
    // EXTEND_CLIMBER.onFalse(climb.stopCommand());

    // RETRACT_CLIMBER.whileTrue(climb.retractCommand());
    // RETRACT_CLIMBER.onFalse(climb.stopCommand());

    // Intake Controls
    // INTAKE.whileTrue(intake.intakeCommand());
    // INTAKE.onFalse(intake.stopCommand());
    // INTAKE.whileTrue(hopper.runContinuous());
    // INTAKE.onFalse(hopper.stopCommand());
    EXTEND_INTAKE.onTrue(intake.deployCommand());
    RETRACT_INTAKE.onTrue(intake.retractCommand());

    // HopperTrigger.whileTrue(hopper.runContinuous());
    // HopperTrigger.onFalse(hopper.stopCommand());

    // HopperStopTrigger.onTrue(hopper.stopCommand());
    // stopFlyWheelTrigger.onTrue(shooter.stopCommand());



    // Add command scheduler to SmartDashboard for debugging
    SmartDashboard.putData("commandscheduler", CommandScheduler.getInstance());

    // Set default drive command - field-relative joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    // Button bindings
    // Reset gyro when button is pressed
    RESET_GYRO.onTrue(
        new InstantCommand(
            () -> {
              drive.resetYaw();
            },
            drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}