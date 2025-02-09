package frc.robot.commands;


import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;


public class ScoreCoral extends SequentialCommandGroup{


  public ScoreCoral(Elevator elevator, Wrist wrist, Coral coral, double coralLevel, Boolean scoreConfirm) {

    addRequirements(elevator, wrist, coral);

    addCommands(

        // new InstantCommand(() -> new DriveToPose(drive, () -> new Pose2d())),
        
        wrist.setWristAngle(-15),
        elevator.moveTo(coralLevel),

        coral.moveSpeed(0.375).onlyWhile(()->scoreConfirm)
        

    );
  }
  
}