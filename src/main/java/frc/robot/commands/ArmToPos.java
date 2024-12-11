package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPos extends Command {
    private final ArmSubsystem armSubsystem;
    private double setpoint;
    private double allowed_error;


    public ArmToPos (ArmSubsystem armSubsystem, double setpoint){
        this.armSubsystem = armSubsystem;
        this.setpoint = setpoint;
        this.allowed_error = 0.05;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        FeedbackConfigs fdb = armSubsystem.getFlipperConfig().Feedback;
        fdb.SensorToMechanismRatio = 85.33; // 12.8 rotor rotations per mechanism rotation

        MotionMagicConfigs mm = armSubsystem.getFlipperConfig().MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(50));


        Slot0Configs slot0 = armSubsystem.getFlipperConfig().Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    }

    @Override
    public void execute() {
        armSubsystem.getFlipper().setControl(armSubsystem.getMMVCont().withPosition(setpoint));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (armSubsystem.getFlipper().getPosition().getValueAsDouble() < setpoint+allowed_error && 
            armSubsystem.getFlipper().getPosition().getValueAsDouble() > setpoint-allowed_error ){
                return true;
            }
        return false;
    }

}