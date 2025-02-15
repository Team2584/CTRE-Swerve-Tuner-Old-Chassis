// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Controller imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import frc.robot.commands.*;
import frc.robot.commands.NeutralAlgae;
import frc.robot.commands.PickupReefAlgae;
import frc.robot.commands.ScoreCoral;

import frc.robot.commandgroup.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {
  private double governor = 0.35; // Added to slow MaxSpeed to 35%. Set to 1 for full speed.
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1 * Math.PI).in(RadiansPerSecond); // 1/2 of a rotation per
                                                                                           // second max angular
                                                                                           // velocity

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Joystick buttonBoard = new Joystick(2);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final Field2d m_field = new Field2d();

  private ElevatorSubsystem buildElevatorSubsystem() {
    return new ElevatorSubsystem();
  }

  private AlgaeSubsystem buildAlgaeMech() {
    return new AlgaeSubsystem();
  }

  private CoralSubsystem buildCoralMech() {
    return new CoralSubsystem();
  }

  private WristSubsystem buildWrist() {
    return new WristSubsystem();
  }

  private ClimberSubsystem buildClimberSubsystem() {
    return new ClimberSubsystem();
  }

  public CommandSwerveDrivetrain getDrivetrain(){
    return drivetrain;
  }

  public ElevatorSubsystem getElevator() {
    return elevator;
  }

  public AlgaeSubsystem getAlgaeMech() {
    return algae;
  }

  public CoralSubsystem getCoral() {
    return coral;
  }

  public WristSubsystem getWrist() {
    return wrist;
  }

  public ClimberSubsystem getClimber() {
    return climber;
  }

  public CommandXboxController getJoystick(){
    return joystick;
  }
  
  
  private final AlgaeSubsystem algae = buildAlgaeMech();
  private final CoralSubsystem coral = buildCoralMech();
  private final WristSubsystem wrist = buildWrist();
  private final ElevatorSubsystem elevator = buildElevatorSubsystem();
  private final ClimberSubsystem climber = buildClimberSubsystem();

  // Map buttons to trigger variables
  private final JoystickButton redL4 = new JoystickButton(buttonBoard, 5);
  private final JoystickButton redL3 = new JoystickButton(buttonBoard, 6);
  private final JoystickButton redL2 = new JoystickButton(buttonBoard, 7);
  private final JoystickButton redL1 = new JoystickButton(buttonBoard, 8);

  private final JoystickButton blue4 = new JoystickButton(buttonBoard, 9);
  private final JoystickButton blue3 = new JoystickButton(buttonBoard, 10);
  private final JoystickButton blue2 = new JoystickButton(buttonBoard, 11);
  private final JoystickButton blue1 = new JoystickButton(buttonBoard, 12);

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("line");

    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Field", m_field);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * governor) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * governor) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate * governor) // Drive counterclockwise with
                                                                                   // negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
    // -joystick.getLeftX()))
    // ));

    // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(0.5).withVelocityY(0))
    // );
    // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    // );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    redL4.onTrue(
      new ScoreCoralState(this,L4)).onFalse(new NeutralState(this)
    );
    redL3.onTrue(
      new ScoreCoralState(this,L3)).onFalse(new NeutralState(this)
    );
    redL2.onTrue(
      new ScoreCoralState(this,L2)).onFalse(new NeutralState(this)
    );
    redL1.onTrue(
      new ScoreCoralState(this,L1)).onFalse(new NeutralState(this)
    );
    // redL4.onTrue(
    //     new ScoreCoral(elevator, wrist, coral, L4)).onFalse(new InstantCommand(() -> elevator.setHeight(0)));
    // redL3.onTrue(
    //     new ScoreCoral(elevator, wrist, coral, L3)).onFalse(new InstantCommand(() -> elevator.setHeight(0)));
    // redL2.onTrue(
    //     new ScoreCoral(elevator, wrist, coral, L2)).onFalse(new InstantCommand(() -> elevator.setHeight(0)));
    // redL1.onTrue(
    //     new ScoreCoral(elevator, wrist, coral, L1)).onFalse(new InstantCommand(() -> elevator.setHeight(0)));

    blue4.onTrue(
        new NetAlgae(this)).onFalse(new NeutralState(this));

    blue3.onTrue(
        new PickupReefAlgaeState(this,ALGAE_HIGH).until(() -> algae.holdingAlgae())
            .finallyDo(()->new NeutralAlgae(this)))
        .onFalse(new NeutralAlgae(this));

    blue2.onTrue(
        new PickupReefAlgaeState(this,ALGAE_LOW).until(() -> algae.holdingAlgae())
          .finallyDo(()->new NeutralAlgae(this)))
        .onFalse(new NeutralAlgae(this));

    blue1.onTrue(
        new PickupReefAlgae(elevator,wrist,algae,GROUND_ALGAE,30).until(() -> algae.holdingAlgae())
          .finallyDo(()->new NeutralAlgae(this)))
        .onFalse(new NeutralAlgae(this));

    // joystick.leftTrigger().whileTrue(new InstantCommand(() ->
    // elevator.setHeight(ElevatorConstants.L3)));
    // joystick.leftTrigger().whileTrue(new InstantCommand(() ->
    // elevator.setHeight(ElevatorConstants.L3)));
    // joystick.leftTrigger().whileTrue(new InstantCommand(() ->
    // elevator.setHeight(ElevatorConstants.L3)));

    // joystick.leftBumper().onTrue(new
    // ScoreCoral(elevator,wrist,coral,L1)).onFalse(new
    // InstantCommand(()->elevator.setHeight(0)));
    // joystick.leftTrigger().onTrue(new
    // ScoreCoral(elevator,wrist,coral,L2)).onFalse(new
    // InstantCommand(()->elevator.setHeight(0)));
    // joystick.rightBumper().onTrue(new
    // ScoreCoral(elevator,wrist,coral,L3)).onFalse(new
    // InstantCommand(()->elevator.setHeight(0)));
    // joystick.rightTrigger().onTrue(new
    // ScoreCoral(elevator,wrist,coral,L4)).onFalse(new
    // InstantCommand(()->elevator.setHeight(0)));

    joystick.rightBumper().onTrue(new IntakeCoral(this));
    joystick.rightTrigger().whileTrue(new ParallelCommandGroup(coral.shootCoral(), algae.outtakeCommand()));

    joystick.leftTrigger().whileTrue(new driveWithSpeed(drivetrain,joystick,0.2));

    joystick.povRight().whileTrue(climber.lowerRobot());
    joystick.povLeft().whileTrue(climber.liftRobot());



    // joystick.povDown().onTrue(new
    // ReefAlgae(elevator,wrist,algae,GROUND_ALGAE).until(()->algae.holdingAlgae())).onFalse(new
    // ParallelCommandGroup(new InstantCommand(()->elevator.setHeight(0)),new
    // InstantCommand(()->algae.setClawSpeed(0)),wrist.WristPose(-50)));
    // joystick.povLeft().onTrue(new
    // ReefAlgae(elevator,wrist,algae,ALGAE_LOW).until(()->algae.holdingAlgae())).onFalse(new
    // ParallelCommandGroup(new InstantCommand(()->elevator.setHeight(0)),new
    // InstantCommand(()->algae.setClawSpeed(0)),wrist.WristPose(-50)));
    // joystick.povUp().onTrue(new
    // ReefAlgae(elevator,wrist,algae,ALGAE_HIGH).until(()->algae.holdingAlgae())).onFalse(new
    // ParallelCommandGroup(new InstantCommand(()->elevator.setHeight(0)),new
    // InstantCommand(()->algae.setClawSpeed(0)),wrist.WristPose(-50)));

    // joystick.povDown().whileTrue(wrist.runOnce(()->wrist.setWristSpeed(0.1)));
    // joystick.rightTrigger().toggleOnTrue(new ArmToPos(getFlipper(),
    // -0.47)).toggleOnFalse(new ArmToPos(getFlipper(), 0));
    // joystick.leftBumper().whileTrue(new ArmToPos(getFlipper(),-0.25)).onFalse(new
    // ArmToPos(getFlipper(),0));
    // joystick.leftBumper().and(joystick.rightBumper()).whileTrue(new
    // OuttakeBucket(getClaw()));
    // joystick.x().whileTrue(getClaw().runOnce(() ->
    // getClaw().setClawSpeed(0.15))).whileFalse(getClaw().runOnce(() ->
    // getClaw().setClawSpeed(0)));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }
}