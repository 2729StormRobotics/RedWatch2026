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

import java.util.List;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOReal;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOReal;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.util.PowerOrchestration;
import frc.robot.util.drive.DriveControls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Project Titan Robot Container.
 * Orchestrates 17 motors across Drive, Shooter, Intake, Kicker, Hopper, and
 * Climb.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Kicker kicker;
  private final Hopper hopper;
  private final Climb climb;

  // Power Management
  private final PowerOrchestration powerOrchestration = PowerOrchestration.getInstance();

  // SimforMAPLE
  private SwerveDriveSimulation driveSimulation = null;

  // LEDs
  private final BlinkinLEDController ledController = BlinkinLEDController.getInstance();

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d field;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // Create vision IO first (dummy for Drive constructor - Vision subsystem
        // handles actual vision)
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3),
            (pose) -> {
            });

        vision = new Vision(
            new VisionIOLimelight(VisionConstants.LEFT_LIMELIGHT_NAME),
            new VisionIOLimelight(VisionConstants.RIGHT_LIMELIGHT_NAME),
            drive);

        shooter = new Shooter(new FlywheelIOReal(), new HoodIOReal(), new TurretIOReal(), drive, driveSimulation);
        intake = new Intake(new IntakeIOReal());
        kicker = new Kicker(new KickerIOReal(), shooter);
        hopper = new Hopper(new HopperIOReal());
        climb = new Climb(new ClimbIOReal());
        break;

      case SIM:

        edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId(Constants.SIM_STATION_ID);
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
            new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        // Create vision IO first (dummy for Drive constructor - Vision subsystem
        // handles actual vision)
        drive = new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(driveSimulation.getModules()[0]),
            new ModuleIOSim(driveSimulation.getModules()[1]),
            new ModuleIOSim(driveSimulation.getModules()[2]),
            new ModuleIOSim(driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);

        vision = new Vision(new VisionIOSim(), new VisionIOSim(), drive);
        shooter = new Shooter(new FlywheelIOSim(), new HoodIOSim(), new TurretIOSim(), drive, driveSimulation);
        intake = new Intake(new IntakeIOSim(driveSimulation));
        kicker = new Kicker(new KickerIOSim(), shooter);
        hopper = new Hopper(new HopperIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            (pose) -> {
            });
        // sampleMotor = new SampleMotor(new SampleMotorIO() {});

        // Vision subsystem with empty IO for replay
        vision = new Vision(
            new VisionIO() {
            },
            new VisionIO() {
            },
            drive);
        shooter = new Shooter(new FlywheelIO() {
        }, new HoodIO() {
        }, new TurretIO() {
        }, drive, driveSimulation);
        intake = new Intake(new IntakeIO() {
        });
        kicker = new Kicker(new KickerIO() {
        }, shooter);
        hopper = new Hopper(new HopperIO() {
        });
        climb = new Climb(new ClimbIO() {
        });
        break;
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Register Named Commands
    NamedCommands.registerCommand("ClimbExtend", climb.climbCommand());
    NamedCommands.registerCommand("ClimbRetract", climb.retractCommand());
    NamedCommands.registerCommand("IntakeDeploy", intake.deployCommand());
    NamedCommands.registerCommand("IntakeRetract", intake.retractCommand());
    NamedCommands.registerCommand("PrepShooter", shooter.prepCommand());
    NamedCommands.registerCommand("StartShots5seconds", shooter.shootforseconds(intake, kicker, 5));

    // Logging callbacks for PathPlanner
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
      Logger.recordOutput("PathPlanner/RobotPose", pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
      Logger.recordOutput("PathPlanner/TargetPose", pose);
    });

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  public void updatePowerOrchestration() {
    powerOrchestration.periodic();
  }

  private void configureButtonBindings() {
    // Configure controls based on active driver profile (e.g., Krithik)
    DriveControls.configureControls();

    ledController.orange();

    // --- Drive Bindings ---
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    RESET_GYRO.onTrue(new InstantCommand(drive::resetYaw, drive));

    // "The Brick" mode - locks swerve modules to resist defense
    DRIFT_BRACE.whileTrue(Commands.run(drive::stopWithX, drive));

    // --- Intake & Hopper ("Touch it, Own it" + "Fullness Detection") ---
    INTAKE_COLLECT.whileTrue(
        Commands.run(intake::deploy, intake)
            .andThen(Commands.runOnce(intake::intake, intake).withTimeout(2))
            .andThen(Commands.runOnce(intake::retract, intake))
            .andThen(Commands.runOnce(() -> m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(Commands.runOnce(() -> m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))));

    INTAKE_EJECT.whileTrue(
        Commands.parallel(
            Commands.run(intake::retract, intake),
            hopper.runCommand(-8.0),
            Commands.run(kicker::reverse, kicker)));

    HOPPER_AGITATE.whileTrue(hopper.runCommand(8.0));

    // --- Shooter & Scoring (AutoScore + CRT Resets) ---

    // Main Scoring Command: Wait for shooter readiness, then fire kicker
    AUTO_SCORE.whileTrue(shooter.autoScoreCommand(intake, kicker));

    // Toggle Move-and-Shoot (Vector Compensation)
    MOVE_AND_SHOOT.onTrue(new InstantCommand(() -> shooter.enableMoveAndShoot()));
    MOVE_AND_SHOOT.onFalse(new InstantCommand(() -> shooter.disableMoveAndShoot()));
    MANUAL_SHOOT.whileTrue(Commands.run(kicker::fire, kicker));

    // --- Climb ---
    CLIMB_SEQUENCE.whileTrue(climb.climbCommand().withTimeout(1));
    CLIMB_RETRACT.whileTrue(climb.retractCommand().withTimeout(1));
  }

  public void reset() {
    drive.resetYaw();
    shooter.setTurretAngle(0.0);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // SIM STUFF

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;
    boolean isRed = getAlliance() == Alliance.Red;


    Logger.recordOutput("FieldSimulation/Alliance", isRed);

    // Start 3 meters from the Blue wall, facing 0 degrees
    Pose2d startPose = new Pose2d(3.0, 3.0, new Rotation2d(0));

    if (isRed) {
      // Mirror the X coordinate and flip the rotation by 180 degrees
      startPose = new Pose2d(
          FieldConstants.fieldLength - startPose.getX(),
          startPose.getY(),
          startPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    drive.setPose(startPose);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

    /** * Returns the current alliance, defaulting to the simulation constant if FMS is disconnected.
   */
  private Alliance getAlliance() {
      return DriverStation.getAlliance().orElse(DriverStationSim.getAllianceStationId() == AllianceStationID.Red1 ? Alliance.Red : Alliance.Blue);
  }
}
