package frc.robot.commands;


import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.wrist.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.coral.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;


public class ScoreCoral extends ParallelCommandGroup{


  public ScoreCoral(Drive drive, Elevator elevator, Wrist wrist, Coral coral, double coralLevel, BooleanSupplier scoreConfirm) {

    addRequirements(drive, elevator, wrist, coral);

    addCommands(

        //DRIVE COMMAND THAT AUTO ALIGNS ROBOT TO VISION TARGET,
        
        wrist.moveWrist(0.27), 

        elevator.moveTo(coralLevel),

        coral.moveSpeed(0.375).onlyWhile(scoreConfirm)

    );
  }
  
}