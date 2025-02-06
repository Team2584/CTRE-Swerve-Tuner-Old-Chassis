// Copyright 2021-2025 FRC 6328
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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.wrist.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climb;
  private final Intake intake;
  private final Coral coral;
  private final Wrist wrist;
  private final Elevator elevator;
  
  //Motor Music for funsies
  // Orchestra all_orchestra = new Orchestra(); //Make and orchestra!
  // boolean isPlaying = false;


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Add all motors to your orchestm_orchestra.addInstrument(leader); //Add instrument to your music orchestra
    //all_orchestra.addInstrument(new TalonFX(ElevatorConstants.ELEVATOR_LEFT_ID),0);
    //all_orchestra.addInstrument(new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_ID),0);
    //all_orchestra.addInstrument(new TalonFX(ClimberConstants.leftClimberCanId),0);
    //all_orchestra.addInstrument(new TalonFX(ClimberConstants.rightClimberCanId),0);
    //all_orchestra.addInstrument(new TalonFX(IntakeConstants.INTAKE_ID),0);
    //all_orchestra.addInstrument(new TalonFX(IntakeConstants.WRIST_ID),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.BackLeft.DriveMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.BackLeft.SteerMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.BackRight.DriveMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.BackRight.SteerMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.FrontLeft.DriveMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.FrontLeft.SteerMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.FrontRight.DriveMotorId),0);
    // all_orchestra.addInstrument(new TalonFX(TunerConstants.FrontRight.SteerMotorId),0);




    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        climb = new Climber(new ClimberIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());
        coral = new Coral(new CoralIOTalonFX());
        
        break;


      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        climb = new Climber(new ClimberIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());
        coral = new Coral(new CoralIOSim());
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

        climb = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        coral = new Coral(new CoralIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    //controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Elevator Scoring Controls for Coral
    // controller.povUp().whileTrue(elevator.moveTo(ElevatorConstants.L1)); //povUp mapped to Top Left paddle
    // controller.povRight().whileTrue(elevator.moveTo(ElevatorConstants.L1)); //povRight mapped to Bottom Left paddle
    // controller.povDown().whileTrue(elevator.moveTo(ElevatorConstants.L1)); //povDown mapped to Top Right paddle
    // controller.povLeft().whileTrue(elevator.moveTo(ElevatorConstants.L1)); //povLeft mapped to Bottom Right paddle

    // controller.rightBumper().whileTrue(elevator.resetHeight()); //Coral Station intake/ Processor

    // controller.leftTrigger().whileTrue(elevator.resetHeight()); //Algae reef intake

    // controller.rightTrigger().whileTrue(elevator.resetHeight()); // Score Confirm




    // while (controller.rightBumper().getAsBoolean()){

    //   controller.rightBumper().whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //       drive,
    //       () -> -controller.getLeftY(),
    //       () -> -controller.getLeftX(),
    //       () -> new Rotation2d()));

    
    //   controller.a().whileTrue(elevator.moveTo(ElevatorConstants.L1));
    //   controller.b().whileTrue(elevator.moveTo(ElevatorConstants.L2));
    //   controller.x().whileTrue(elevator.moveTo(ElevatorConstants.L3));
    //   controller.y().whileTrue(elevator.moveTo(ElevatorConstants.L4));


    // }
    

    
    controller.start().whileTrue(coral.moveSpeed(0.375));

    // Elevator command triggers
    controller.povUp().whileTrue(wrist.moveWrist(-5));
    controller.povDown().whileTrue(wrist.moveWrist(5));
    
    controller.povLeft().whileTrue(climb.runPercent(50));
    controller.povRight().whileTrue(climb.runPercent(-50));

    controller.x().whileTrue(elevator.moveTo(ElevatorConstants.HOME));

    controller.leftTrigger().onTrue(elevator.moveTo(ElevatorConstants.L1));
    controller.leftBumper().onTrue(elevator.moveTo(ElevatorConstants.L2));
    controller.rightTrigger().onTrue(elevator.moveTo(ElevatorConstants.L3));
    controller.rightBumper().onTrue(elevator.moveTo(ElevatorConstants.L4));
    
    

    controller.y().whileTrue(elevator.runPercent(0.1))//make button
                .onFalse(elevator.runPercent(0.0));

    controller.a().whileTrue(elevator.runPercent(-0.1))//make button
                .onFalse(elevator.runPercent(0.0));

    //controller.povRight().whileTrue(elevator.resetHeight());

    //controller.povRight().whileTrue(intake.outtakeCommand(30));
    controller.b().whileTrue(intake.intakeCommand(30));
                


    // controller.povLeft().onTrue(playMusicAll("Undertale.chrp")); // make button

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // public Command playMusicAll(String fileNeame){
  //   all_orchestra.loadMusic(fileNeame);
  //    return Commands.runOnce(() -> all_orchestra.play());
  //   } 
  
}
