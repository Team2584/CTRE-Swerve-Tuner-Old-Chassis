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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.Constants.*;
import frc.robot.commands.GoToNeutral;
import frc.robot.commands.ReefAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.WristToPos;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private double governor = 0.35; // Added to slow MaxSpeed to 35%. Set to 1 for full speed.
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1*Math.PI).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    private final CommandXboxController joystick = new CommandXboxController(0);

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

    private ElevatorSubsystem getElevator() {
      return elevator;
    }

    private AlgaeSubsystem getAlgaeMech() {
        return algae;
    }

    private CoralSubsystem getCoral() {
      return coral;
  }

    private WristSubsystem getWrist() {
        return wrist;
    }

    private final AlgaeSubsystem algae = buildAlgaeMech();
    private final CoralSubsystem coral = buildCoralMech();
    private final WristSubsystem wrist = buildWrist();
    private final ElevatorSubsystem elevator = buildElevatorSubsystem();



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
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * governor) // Drive forward with negative Y (forward)
                  .withVelocityY(-joystick.getLeftX() * MaxSpeed * governor) // Drive left with negative X (left)
                  .withRotationalRate(-joystick.getRightX() * MaxAngularRate * governor) // Drive counterclockwise with negative X (left)
          )
      );

    //   joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
      // joystick.b().whileTrue(drivetrain.applyRequest(() ->
      //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
      // ));

      // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
      //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
      // );
      // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
      //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
      // );

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      // reset the field-centric heading on left bumper press
      //joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      // joystick.leftTrigger().whileTrue(new InstantCommand(() -> elevator.setHeight(ElevatorConstants.L3)));
      joystick.a().whileTrue(wrist.WristPose(-80));
      joystick.b().whileTrue(wrist.WristPose(-90));
      joystick.x().onTrue(new ScoreCoral(elevator,wrist,coral,Constants.ElevatorConstants.L3)).onFalse(new InstantCommand(()->elevator.setHeight(0)));
      joystick.y().onTrue(new ScoreCoral(elevator,wrist,coral,Constants.ElevatorConstants.L4)).onFalse(new InstantCommand(()->elevator.setHeight(0)));
      joystick.rightTrigger().whileTrue(coral.shootCoral());
      joystick.leftBumper().onTrue(coral.intakeCoral());
      joystick.povUp().onTrue(new ReefAlgae(elevator,wrist,algae,Constants.ElevatorConstants.L3)).onFalse(new InstantCommand(()->elevator.setHeight(0)).andThen(new InstantCommand(()->algae.setClawSpeed(0))));

      //joystick.povDown().whileTrue(wrist.runOnce(()->wrist.setWristSpeed(0.1)));
      // joystick.rightTrigger().toggleOnTrue(new ArmToPos(getFlipper(), -0.47)).toggleOnFalse(new ArmToPos(getFlipper(), 0));
      // joystick.leftBumper().whileTrue(new ArmToPos(getFlipper(),-0.25)).onFalse(new ArmToPos(getFlipper(),0));
      // joystick.leftBumper().and(joystick.rightBumper()).whileTrue(new OuttakeBucket(getClaw()));
      // joystick.x().whileTrue(getClaw().runOnce(() -> getClaw().setClawSpeed(0.15))).whileFalse(getClaw().runOnce(() -> getClaw().setClawSpeed(0)));

    
      drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
      /* First put the drivetrain into auto run mode, then run the auto */
      return autoChooser.getSelected();
  }
}