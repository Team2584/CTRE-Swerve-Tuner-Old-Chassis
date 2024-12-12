package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeBucket extends Command {
    private final ArmSubsystem armSubsystem;
    public IntakeBucket (ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        armSubsystem.setClawSpeed(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setClawSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (armSubsystem.holdingBucket()){
            return true;
        }
        return false;
    }

}