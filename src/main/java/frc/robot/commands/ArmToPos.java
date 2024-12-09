package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPos extends Command {
    private final ArmSubsystem armSubsystem;
    private double setpoint;


    public ArmToPos (ArmSubsystem armSubsystem, double setpoint){
        this.armSubsystem = armSubsystem;
        this.setpoint = setpoint;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}