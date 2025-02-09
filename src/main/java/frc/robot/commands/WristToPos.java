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
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristToPos extends Command {
    private final WristSubsystem wristSubsystem;
    private double setpoint;
    private double allowed_error;


    public WristToPos (WristSubsystem wristSubsystem, double setpoint){
        this.wristSubsystem = wristSubsystem;
        this.setpoint = setpoint;
        this.allowed_error = 0.05;

        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize(){
        FeedbackConfigs fdb = wristSubsystem.getWristConfig().Feedback;
        fdb.SensorToMechanismRatio = 85.33; // X motor rotations per mechanism rotation

        MotionMagicConfigs mm = wristSubsystem.getWristConfig().MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(12))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(8))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(60));


        Slot0Configs slot0 = wristSubsystem.getWristConfig().Slot0;
        slot0.kS = 0.10; // volts
        slot0.kV = 0.1; // volts * seconds / distance
        slot0.kA = 0.01; // volts * seconds^2 / distance
        slot0.kP = 12;  // proportional
        slot0.kI = 0;    // integral
        slot0.kD = 0.8;  // derivative
        wristSubsystem.getWristMotor().getConfigurator().apply(wristSubsystem.getWristConfig());
    }

    @Override
    public void execute() {
        wristSubsystem.getWristMotor().setControl(wristSubsystem.getMMVCont().withPosition(setpoint));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (wristSubsystem.getWristMotor().getPosition().getValueAsDouble() < setpoint+allowed_error && 
            wristSubsystem.getWristMotor().getPosition().getValueAsDouble() > setpoint-allowed_error ){
                return true;
            }
        return false;
    }

}