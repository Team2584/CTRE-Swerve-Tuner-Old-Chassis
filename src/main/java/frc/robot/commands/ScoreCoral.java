package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class ScoreCoral extends SequentialCommandGroup {

  public ScoreCoral(ElevatorSubsystem elevator, WristSubsystem wrist, CoralSubsystem coral, double coralLevel) {

    addRequirements(wrist, coral, elevator);

    addCommands(

        // new InstantCommand(() -> new DriveToPose(drive, () -> new Pose2d())),

        new ParallelCommandGroup(wrist.WristPose(-45),
        new InstantCommand(()->elevator.setHeight(coralLevel))),
        wrist.WristPose(-75)


    );
    
  }

}