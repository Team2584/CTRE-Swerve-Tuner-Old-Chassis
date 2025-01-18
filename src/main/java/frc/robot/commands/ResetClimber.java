package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ResetClimber extends Command {
    private final Climber climber;
    
    public ResetClimber (Climber climber){
        this.climber = climber;
        
        addRequirements(climber);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        climber.setClimberSpeed(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}