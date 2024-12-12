package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class ArmFlipLower extends Command {
    private final ArmSubsystem armSubsystem;
    private final ArmToPos armToPosCommand; // hold a reference to the command
    private boolean armToPoseSchedueled = false; // To ensure we schedule it only once


    public ArmFlipLower (ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        this.armToPosCommand = new ArmToPos(armSubsystem, -0.45);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armToPoseSchedueled = false;
    }

    @Override
    public void execute() {
        if (!armToPoseSchedueled){
            armToPosCommand.schedule();
            armToPoseSchedueled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        
        return armToPosCommand.isFinished();
    }

}