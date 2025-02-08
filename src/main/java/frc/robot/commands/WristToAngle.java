package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristIOTalonFX;

public class WristToAngle extends Command {
    private final Wrist wrist;
    private double setpointAngle;
    private double allowed_error;


    public WristToAngle (Wrist wrist, double setpointAngle){
        this.wrist = wrist;
        this.setpointAngle = setpointAngle;
        this.allowed_error = 1.5;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        FeedbackConfigs fdb = wrist.getWristConfigs().Feedback;
        fdb.SensorToMechanismRatio = WristConstants.WRIST_GEAR_RATIO; // X motor rotations per mechanism rotation

        wrist.getWristConfigs().MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wrist.getWristConfigs().MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        MotionMagicConfigs motionMagicConfigs = wrist.getWristConfigs().MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 5; // Rotations per second
        motionMagicConfigs.MotionMagicAcceleration=  8; // Rotations per second^2
        motionMagicConfigs.MotionMagicJerk =  40; // Rotations per second ^3
        
        
        Slot0Configs slot0 = wrist.getWristConfigs().Slot0;
        slot0.kS = 0.30; // volts
        slot0.kV = 0.1; // volts * seconds / distance
        slot0.kA = 0.01; // volts * seconds^2 / distance
        slot0.kP = 33;  // proportional
        slot0.kI = 0;    // integral
        slot0.kD = 0.8;  // derivative
        wrist.getWristMotor().getConfigurator().apply(wrist.getWristConfigs());

    }

    @Override
    public void execute() {
        wrist.getWristMotor().setControl(wrist.getMMVCont().withPosition(setpointAngle));
        }

    @Override
    public void end(boolean interrupted) {
        wrist.getCurrentCommand().cancel();
    }

    public boolean isFinished() {
        if (Units.rotationsToDegrees(wrist.getWristMotor().getPosition().getValueAsDouble()) < setpointAngle+allowed_error && 
            Units.rotationsToDegrees(wrist.getWristMotor().getPosition().getValueAsDouble()) > setpointAngle-allowed_error ){
                return true;
            }
        return false;
    }
}

