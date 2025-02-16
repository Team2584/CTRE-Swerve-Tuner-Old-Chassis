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
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class GoToNeutral extends SequentialCommandGroup {

  public GoToNeutral(ElevatorSubsystem elevator, WristSubsystem wrist, AlgaeSubsystem algae) {

    addRequirements(wrist, algae, elevator);

    addCommands(

        // new InstantCommand(() -> new DriveToPose(drive, () -> new Pose2d())),
        
   
        new InstantCommand(()->elevator.setHeight(0)),
        new InstantCommand(()-> algae.setClawSpeed(0)),
        wrist.WristPose(-90)





    );
    
  }

}