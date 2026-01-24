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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LED.BlinkinLEDController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Project Titan Robot Container.
 * Orchestrates 17 motors across Drive, Shooter, Intake, Kicker, Hopper, and Climb.
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
        // Create vision IO first (dummy for Drive constructor - Vision subsystem handles actual vision)
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        
        vision = new Vision(
                new VisionIOLimelight(VisionConstants.LEFT_LIMELIGHT_NAME),
                new VisionIOLimelight(VisionConstants.RIGHT_LIMELIGHT_NAME),
                drive);
        
        shooter = new Shooter(new FlywheelIOReal(), new HoodIOReal(), new TurretIOReal(), drive);
        intake = new Intake(new IntakeIOReal());
        kicker = new Kicker(new KickerIOReal(), shooter);
        hopper = new Hopper(new HopperIOReal());
        climb = new Climb(new ClimbIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // Create vision IO first (dummy for Drive constructor - Vision subsystem handles actual vision)
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        
        vision = new Vision(new VisionIOSim(), new VisionIOSim(), drive);
        shooter = new Shooter(new FlywheelIOSim(), new HoodIOSim(), new TurretIOSim(), drive);
        intake = new Intake(new IntakeIOSim());
        kicker = new Kicker(new KickerIOSim(), shooter);
        hopper = new Hopper(new HopperIOSim());
        climb = new Climb(new ClimbIOSim());
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
        // sampleMotor = new SampleMotor(new SampleMotorIO() {});
        
        // Vision subsystem with empty IO for replay
        vision =
            new Vision(
                new VisionIO() {},
                new VisionIO() {},
                drive);
        shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {}, new TurretIO() {}, drive);
        intake = new Intake(new IntakeIO() {});
        kicker = new Kicker(new KickerIO() {}, shooter);
        hopper = new Hopper(new HopperIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

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
        Commands.sequence(
            Commands.runOnce(intake::deploy, intake),
            Commands.parallel(
                Commands.run(intake::intake, intake),
                hopper.runCommand(8.0)
            )
        ).until(intake::isGamePieceDetected)
         .andThen(Commands.runOnce(intake::retract, intake))
         .andThen(Commands.runOnce(() -> m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
         .andThen(Commands.waitSeconds(0.5))
         .andThen(Commands.runOnce(() -> m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)))
    );

    INTAKE_EJECT.whileTrue(
        Commands.parallel(
            Commands.run(intake::eject, intake),
            hopper.runCommand(-8.0),
            Commands.run(kicker::reverse, kicker)
        )
    );

    HOPPER_AGITATE.whileTrue(hopper.runCommand(8.0));

    // --- Shooter & Scoring (AutoScore + CRT Resets) ---
    
    // Main Scoring Command: Wait for shooter readiness, then fire kicker
    AUTO_SCORE.whileTrue(
        Commands.parallel(
            Commands.run(() -> {
                // Coordinate aim uses robot pose to calculate heading to hub
                shooter.enableMoveAndShoot();
            }, shooter),
            Commands.sequence(
                Commands.waitUntil(shooter::isReadyToFire),
                kicker.fireCommand()
            )
        )
    );

    // Toggle Move-and-Shoot (Vector Compensation)
    MOVE_AND_SHOOT.onTrue(new InstantCommand(() -> shooter.enableMoveAndShoot()));
    MOVE_AND_SHOOT.onFalse(new InstantCommand(() -> shooter.disableMoveAndShoot()));

    // Manual CRT Turret Reset
    RESET_TURRET.onTrue(new InstantCommand(() -> shooter.setTurretAngle(0.0), shooter));

    MANUAL_SHOOT.whileTrue(Commands.run(kicker::fire, kicker));

    // --- Climb ---
    CLIMB_SEQUENCE.whileTrue(Commands.run(climb::climb, climb));
  }

  public void reset() {
    drive.resetYaw();
    shooter.setTurretAngle(0.0);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}