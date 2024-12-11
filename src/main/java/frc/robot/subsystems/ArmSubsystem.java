package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX m_intake;
    private final TalonFX m_flip;

    private static final TalonFXConfiguration flipperConfig = new TalonFXConfiguration();
    private static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage m_mmReq;
    
    public ArmSubsystem(int intakeCAN, int flipCAN){
        m_intake = new TalonFX(intakeCAN);
        m_flip = new TalonFX(flipCAN);

        m_mmReq = new MotionMagicVoltage(0);
    }

    public TalonFX getIntake(){
        return m_intake;
    }

    public TalonFX getFlipper(){
        return m_flip;
    }

    public TalonFXConfiguration getFlipperConfig(){
        return flipperConfig;
    }

    public TalonFXConfiguration getIntakeConfig(){
        return intakeConfig;
    }

    public MotionMagicVoltage getMMVCont(){
        return m_mmReq;
    }

    public void setClawSpeed(double speed){
        m_intake.set(speed);
    }

    public void setFlipSpeed(double speed){
        m_flip.set(speed);
    }

    public void setSpeed(double speed, String clawflip){
        if (clawflip.toLowerCase().equals("claw")){
            m_intake.set(speed);
        } else if (clawflip.toLowerCase().equals("flip")) {
            m_flip.set(speed);
        }
    }
}
