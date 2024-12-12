package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeBucket extends Command {
    private final ArmSubsystem armSubsystem;
    private final ArmFlipLower armLowerCommand; // hold a reference to the command
    private final ArmFlipUpper armUpperCommand; // hold a reference to the command
    private boolean lowerArmSchedueled = false; // To ensure we schedule it only once
    private boolean raiseArmSchedueled = false; // To ensure we schedule it only once


    public IntakeBucket(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        this.armLowerCommand = new ArmFlipLower(armSubsystem);
        this.armUpperCommand = new ArmFlipUpper(armSubsystem);
        

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
        armSubsystem.setClawSpeed(-0.25);
        if (!lowerArmSchedueled){
            armLowerCommand.schedule();
            lowerArmSchedueled = true;
        }

    }


    @Override
    public void end(boolean interrupted) {
        armSubsystem.setClawSpeed(0);
        if (!raiseArmSchedueled){
            armUpperCommand.schedule();
            raiseArmSchedueled = true;
        }

    }

    @Override
    public boolean isFinished() {
        if (armSubsystem.holdingBucket()){
            return true;
        }
        return false;
    }

}