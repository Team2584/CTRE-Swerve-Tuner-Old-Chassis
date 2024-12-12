package frc.robot.commandgroup;

import frc.robot.commands.ArmToPos;
import frc.robot.commands.IntakeBucket;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class ScoreBucket extends SequentialCommandGroup{

  public ScoreBucket(ArmSubsystem armSubsystem) {

    addRequirements(armSubsystem);

    addCommands(

        new SequentialCommandGroup(new ArmToPos(armSubsystem, -0.47), new IntakeBucket(armSubsystem)), // put arm down

        new ArmToPos(armSubsystem, 0) // put arm up
    );
  }
        
}