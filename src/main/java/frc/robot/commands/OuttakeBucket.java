package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class OuttakeBucket extends Command {
    private final ClawSubsystem clawSubsystem;
    
    public OuttakeBucket (ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;
        
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        clawSubsystem.setClawSpeed(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}