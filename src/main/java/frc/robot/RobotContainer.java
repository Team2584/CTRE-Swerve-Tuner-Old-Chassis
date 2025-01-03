// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.util.AllianceFlipUtil.Flipper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeBucket;
import frc.robot.commands.OuttakeBucket;
import frc.robot.commandgroup.PickupBucket;
import frc.robot.commands.ArmToPos;
import frc.robot.generated.TunerConstants;
import frc.robot.AutoRoutines;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlipperSubsystem;


public class RobotContainer {
    private double governor = 0.35; // Added to slow MaxSpeed to 35%. Set to 1 for full speed.
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1*Math.PI).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
                
    private final Telemetry logger = new Telemetry(MaxSpeed);


     /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private final Field2d m_field = new Field2d();

    private ClawSubsystem buildClaw() {
        return new ClawSubsystem(15);
    }

    private FlipperSubsystem buildFlipper() {
        return new FlipperSubsystem(16);
    }


    private ClawSubsystem getClaw() {
        return claw;
    }

    private FlipperSubsystem getFlipper() {
        return flipper;
    }

    
    private final ClawSubsystem claw = buildClaw();
    private final FlipperSubsystem flipper = buildFlipper();



  public RobotContainer() {

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
      joystick.b().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
      ));

      joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
          forwardStraight.withVelocityX(0.5).withVelocityY(0))
      );
      joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
          forwardStraight.withVelocityX(-0.5).withVelocityY(0))
      );

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      // reset the field-centric heading on left bumper press
      joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      joystick.leftTrigger().whileTrue(new PickupBucket(getFlipper(), getClaw())).onFalse((getClaw().runOnce(() -> getClaw().setClawSpeed(0)).andThen(new ArmToPos(getFlipper(), 0))));
      joystick.rightTrigger().toggleOnTrue(new ArmToPos(getFlipper(), -0.47)).toggleOnFalse(new ArmToPos(getFlipper(), 0));
      joystick.leftBumper().whileTrue(new ArmToPos(getFlipper(),-0.25)).onFalse(new ArmToPos(getFlipper(),0));
      joystick.leftBumper().and(joystick.rightBumper()).whileTrue(new OuttakeBucket(getClaw()));
      joystick.x().whileTrue(getClaw().runOnce(() -> getClaw().setClawSpeed(0.15))).whileFalse(getClaw().runOnce(() -> getClaw().setClawSpeed(0)));



      drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
      /* First put the drivetrain into auto run mode, then run the auto */
      return autoChooser.selectedCommand();
  }
}