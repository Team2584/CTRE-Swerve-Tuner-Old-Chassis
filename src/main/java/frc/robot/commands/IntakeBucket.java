package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class IntakeBucket extends Command {
    private final ClawSubsystem clawSubsystem;
    public IntakeBucket (ClawSubsystem clawSubsystem){
        this.clawSubsystem = clawSubsystem;
        
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        if (!clawSubsystem.holdingBucket()){
            clawSubsystem.setClawSpeed(0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (clawSubsystem.holdingBucket()){
            return true;
        }
        return false;
    }

}