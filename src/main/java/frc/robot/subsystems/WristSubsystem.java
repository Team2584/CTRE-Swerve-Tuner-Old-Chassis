package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.WristConstants;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;


import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX wrist;

    private static final TalonFXConfiguration wristConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage m_mmReq;

    private final VoltageOut vreq;

    private final PositionVoltage posReq;

    public WristSubsystem() {
        wrist = new TalonFX(WristConstants.WRIST_ID);

        m_mmReq = new MotionMagicVoltage(0);

        vreq = new VoltageOut(0);

        posReq = new PositionVoltage(0);

        FeedbackConfigs fdb = wristConfig.Feedback;
        fdb.SensorToMechanismRatio = WristConstants.WRIST_GEAR_RATIO; // X motor rotations per mechanism rotation

            /* Configure Motion Magic */
        MotionMagicConfigs mm = wristConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(12)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(8)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(60));


        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        Slot0Configs slot0 = wristConfig.Slot0;
        slot0.kS = 0.3; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 33; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.8; // A velocity error of 1 rps results in 0.5 V output

        wrist.getConfigurator().apply(wristConfig,0.25);

        wrist.setPosition(-0.25);

    }

     /**
     * Set speed of wrist motor 
     */
    public void setWristSpeed(double speed){
        wrist.set(speed);
    }
    
     /**
     * Gets the wrist motor
     * @return TalonFX
     */
    public TalonFX getWristMotor(){
        return wrist;
    }

    /**
    * Gets the wrist motor configuration
    * @return TalonFXConfiguration
    */
    public TalonFXConfiguration getWristConfig(){
        return wristConfig;
    }

    /**
    * Gets the Motion Magic Profile Request associated with the wrist motor
    * @return MotionMagicVoltage                      
    */
    public MotionMagicVoltage getMMVCont(){
        return m_mmReq;
    }

    public Command WristPose(double angle){
        return runEnd(
            ()->wrist.setControl(m_mmReq.withPosition(Units.degreesToRotations(angle))),
            () -> wrist.setControl(vreq.withOutput(0))).until(()->(Units.rotationsToDegrees(wrist.getPosition().getValueAsDouble()) < angle+5 && 
            Units.rotationsToDegrees(wrist.getPosition().getValueAsDouble()) > angle-5 ));

    }


    @Override
    public void periodic() {
        System.out.println("Wrist Position: " + wrist.getPosition().getValueAsDouble());

    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Wrist voltage: " + wrist.getMotorVoltage());
    }
}
