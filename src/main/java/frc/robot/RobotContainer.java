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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.LED.BlinkinLEDController.BlinkinPattern;
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
import frc.robot.util.drive.AllianceFlipUtil;
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
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
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
import frc.robot.commandgroups.HopperBackwardsIntake;




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
  public final Shooter shooter;
  private final kicker kicker;
  private final Intake intake;
  private final Hopper hopper;

//   private final Climb climber;

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
        // climber = new Climb(new ClimbIOReal());
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
        // climber = new Climb(new ClimbIOSim());
        
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
        // climber = new Climb(new ClimbIOReal());


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

    // Elastic: set TunableNumbers/Shooter/Elastic/DesiredFlywheelRps and DesiredHoodRotations, then run this command to apply both
    // SmartDashboard.putData("Shooter/Elastic/ApplySetpoints", shooter.applyElasticSetpointsCommand());

    System.out.println("[Init] Setting up Path Planner Logging");

    // Logging callback for current robot pose (alliance-relative for driver view)
    PathPlannerLogging.setLogCurrentPoseCallback(
        (unusedPose) -> {
          Pose2d alliancePose = drive.getAlliancePose();
          field.setRobotPose(alliancePose);
          Logger.recordOutput("PathPlanner/RobotPose", alliancePose);
        });

    // Logging callback for target robot pose (alliance-relative)
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Pose2d alliancePose = AllianceFlipUtil.apply(pose);
          field.getObject("target pose").setPose(alliancePose);
          Logger.recordOutput("PathPlanner/TargetPose", alliancePose);
        });

    // Logging callback for the active path, sent as a list of poses (alliance-relative)
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          Pose2d[] alliancePath =
              AllianceFlipUtil.apply(poses.toArray(new Pose2d[0]));
          field.getObject("path").setPoses(alliancePath);
          Logger.recordOutput("PathPlanner/ActivePath", alliancePath);
        });

    // StartShots5seconds
    NamedCommands.registerCommand("StartShots5secondsFar", new ParallelCommandGroup(new WaitCommand(5), new SequentialCommandGroup(new InstantCommand( () -> {shooter.enableMoveAndShoot(); shooter.setFlywheelArmed(false); shooter.setFlywheelVelocity(285);}), Commands.parallel(
            hopper.runContinuous(),
            Commands.run(() -> kicker.setPercent(1), kicker),
            Commands.run(() -> shooter.setFlywheelVelocity(285), shooter)))).withTimeout(5));

            
    NamedCommands.registerCommand("StartShots5seconds", new ParallelCommandGroup(new WaitCommand(5), new SequentialCommandGroup(new InstantCommand( () -> {shooter.enableMoveAndShoot(); shooter.setFlywheelArmed(false); shooter.setFlywheelVelocity(245);}), Commands.parallel(
        hopper.runContinuous(),
        Commands.run(() -> kicker.setPercent(1), kicker),
        Commands.run(() -> shooter.setFlywheelVelocity(245), shooter)))).withTimeout(5));

    // IntakeRetract
    NamedCommands.registerCommand("IntakeRetract", new InstantCommand(() -> intake.retract()));
    //PrepShooter
    NamedCommands.registerCommand("PrepShooter", new InstantCommand(() -> shooter.enableMoveAndShoot()));
    // IntakeDeploy
    NamedCommands.registerCommand("IntakeDeploy", new InstantCommand(() -> intake.deploy()));
    // StartIntake
    NamedCommands.registerCommand("StartIntake", new RepeatCommand(new InstantCommand(() -> intake.intake())).withTimeout(0.1));
    // LockHood
    NamedCommands.registerCommand("LockHood", new InstantCommand(() -> shooter.lockTrench()));
    // UnlockHood
    NamedCommands.registerCommand("UnlockHood", new InstantCommand(() -> shooter.unlockTrench()));
    //Raise Climber
    NamedCommands.registerCommand("RaiseClimber", shooter.decrementPositionCommand());
    // // PullClimber
    NamedCommands.registerCommand("PullClimber", shooter.decrementTestFlywheelVelocityCommand());

    // StartShooting
    NamedCommands.registerCommand("StartShooting", Commands.parallel(
        hopper.runContinuous(),
        Commands.run(() -> kicker.setPercent(1), kicker),
        Commands.run(() -> {shooter.setFlywheelVelocity(250); }, shooter)));
    // StopShooting
    NamedCommands.registerCommand("StopShooting", Commands.parallel(
        hopper.stopCommand(),
        new InstantCommand(() -> kicker.stop(), kicker),
        new InstantCommand(() -> { shooter.setFlywheelArmed(false); shooter.stop(); }, shooter)));
    //Set up auto routines chooser
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

  private void configureButtonBindings() {
    // Configure drive controls based on driver preferences
    DriveControls.configureControls();

    // Set LED to orange on initialization
    ledController.setPattern(BlinkinPattern.BREATH_GRAY);


    // Shooter / Hood: set desired position (periodic applies it; no need to "run" or stop).
    // TICK_2_HOOD.onTrue(shooter.runPositionCommand(2));
    // TICK_37_HOOD.onTrue(shooter.runPositionCommand(37));
    // shooter.runHoodCommand(MOVE_HOOD_JOYSTICK.getAsDouble());

    // Hood manual nudging for calibration / lookup-table data collection.
    // Left bumper: step hood down (toward MIN_POSITION_ROTATIONS).
    // Right bumper: step hood up   (toward MAX_POSITION_ROTATIONS).
    // DecHood.onTrue(shooter.decrementPositionCommand());
    // IncHood.onTrue(shooter.incrementPositionCommand());

    // Flywheel controls: while held, arm flywheel to use lookup-table speed
    // (otherwise it idles at a low speed while MoveAndShoot aiming is active).
    // flyWheelTrigger.whileTrue(
    //     shooter.runTestFlywheelCommand());
    flyWheelTrigger.onTrue(new InstantCommand( () -> shooter.setFlywheelArmed(true)));
    flyWheelTrigger.onFalse(new InstantCommand( () -> shooter.setFlywheelArmed(false)));
    // runFFCharcterization.whileTrue(DriveCommands.feedforwardCharacterization(drive));

    // runWRCharcterization.whileTrue(DriveCommands.wheelRadiusCharacterization(drive));

    // PASS_LOCK.whileTrue(shooter.passCommand());

    // HOOD_DROP_LOCK.whileTrue(shooter.trenchLockCommand());

    // Translator buttons 10/11: bump the test flywheel velocity up/down.
    // INC_TEST_FLYWHEEL.onTrue(shooter.incrementTestFlywheelVelocityCommand());
    // DEC_TEST_FLYWHEEL.onTrue(shooter.decrementTestFlywheelVelocityCommand());

    // reverseFlyWheelTrigger.whileTrue(
    //     Commands.parallel(
    //         Commands.run(() -> shooter.setFlywheelVelocity(-40), shooter),
    //         Commands.run(() -> kicker.setVoltage(-0.5), kicker)));
    // reverseFlyWheelTrigger.onFalse(
    //     Commands.parallel(
    //         Commands.runOnce(shooter::stop, shooter),
    //         Commands.runOnce(kicker::stop, kicker)));

    // turretTrigger0.onTrue(shooter.setTurretAngleDegreesCommand(0.0));
    // turretTrigger180.onTrue(shooter.setTurretAngleDegreesCommand(180.0));
    // turretTrigger45.onTrue(shooter.setTurretAngleDegreesCommand(45.0));
    // turretTrigger90.onTrue(shooter.setTurretAngleDegreesCommand(90.0));

    // Climb Controls
    // EXTEND_CLIMBER.whileTrue(climber.climbCommand());
    // EXTEND_CLIMBER.onFalse(climber.stopCommand());

    // RETRACT_CLIMBER.whileTrue(climber.retractCommand());
    // RETRACT_CLIMBER.onFalse(climber.stopCommand());
    // Intake Controls
    INTAKE_TRIGGER.whileTrue(intake.intakeCommand());
    INTAKE_TRIGGER.onFalse(intake.stopCommand());
    // intake.setDefaultCommand(intake.intakeCommandTrigger(INTAKE));
    EXTEND_INTAKE.onTrue(intake.deployCommand());
    RETRACT_INTAKE.onTrue(intake.retractCommand());

    agitateTrigger.whileTrue(intake.agitateCommand());
    agitateTrigger.onFalse(intake.deployCommand());

    // Translator button 5: run hopper + kicker together
    ReverseHopperTrigger.whileTrue(
        Commands.parallel(
            hopper.runContinuous(),
            Commands.run(() -> kicker.setPercent(1), kicker)));
    ReverseHopperTrigger.onFalse(
        Commands.parallel(
            hopper.stopCommand(),
            Commands.runOnce(kicker::stop, kicker)));

    // Translator button 6: run intake + hopper together
    HopperTrigger.whileTrue(
        Commands.parallel(
            hopper.runContinuous(),
            intake.intakeCommand()));
    HopperTrigger.onFalse(
        Commands.parallel(
            hopper.stopCommand(),
            intake.stopCommand()));

    // HopperOutake.whileTrue(HopperBackwardsIntake.getCommand(intake, hopper, shooter, kicker));
    // HopperOutake.onFalse(HopperBackwardsIntake.getStopCommand(intake, hopper, shooter, kicker));
    
    enableMoveShoot.onTrue(new InstantCommand(() -> shooter.enableMoveAndShoot()));
    disableMoveShoot.onTrue(new InstantCommand(() -> shooter.disableMoveAndShoot()));
    // HopperStopTrigger.onTrue(hopper.stopCommand());
    stopFlyWheelTrigger.onTrue(shooter.stopCommand());



    // Add command scheduler to SmartDashboard for debugging
    SmartDashboard.putData("commandscheduler", CommandScheduler.getInstance());

    // Set default drive command - field-relative joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    // POV on translator: while held, drive with linear input but rotate to face that angle
    POV_UP.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(0)));
    POV_DOWN.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(180)));
    POV_LEFT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(90)));
    POV_RIGHT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(270)));
    // Diagonals (45° intervals)
    POV_UP_RIGHT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(45)));
    POV_DOWN_RIGHT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(135)));
    POV_DOWN_LEFT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(225)));
    POV_UP_LEFT.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive, DRIVE_FORWARD, DRIVE_STRAFE, () -> Rotation2d.fromDegrees(315)));

    reverseKicker.whileTrue(kicker.reverse());
    reverseKicker.onFalse(new InstantCommand(() -> kicker.stop()));
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