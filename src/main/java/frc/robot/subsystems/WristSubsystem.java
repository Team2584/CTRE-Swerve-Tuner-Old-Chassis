package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX wrist;

    private static final TalonFXConfiguration wristConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage m_mmReq;

    public WristSubsystem() {
        wrist = new TalonFX(WristConstants.WRIST_ID);

        m_mmReq = new MotionMagicVoltage(0);
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


    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Wrist voltage: " + wrist.getMotorVoltage());
    }
}
