package frc.robot.commandgroup;

import frc.robot.commands.ArmToPos;
import frc.robot.commands.IntakeBucket;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class PickupBucket extends SequentialCommandGroup{

  public PickupBucket(ArmSubsystem armSubsystem) {

    addRequirements(armSubsystem);

    addCommands(

        new InstantCommand(()->armSubsystem.setClawSpeed(-0.25)), 
        
        new ArmToPos(armSubsystem, -0.47), // put arm down

        new IntakeBucket(armSubsystem),

        new ArmToPos(armSubsystem, 0)

    );
  }
  
}