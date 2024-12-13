package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlipperSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX m_flipper;

    private static final TalonFXConfiguration flipperConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage m_mmReq;

    public FlipperSubsystem(int flipperCANId) {
        m_flipper = new TalonFX(flipperCANId);

        m_mmReq = new MotionMagicVoltage(0);
    }

     /**
     * Set speed of flipper motor 
     */
    public void setFlipperSpeed(double speed){
        m_flipper.set(speed);
    }

    public TalonFX getFlipperMotor(){
        return m_flipper;
    }

    public TalonFXConfiguration getFlipperConfig(){
        return flipperConfig;
    }

    public MotionMagicVoltage getMMVCont(){
        return m_mmReq;
    }


    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Flipper voltage: " + m_flipper.getMotorVoltage());
    }
}
