package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX m_claw;
    private final TalonFX m_flipper;

    private static final TalonFXConfiguration flipperConfig = new TalonFXConfiguration();
    private static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage m_mmReq;

    public ArmSubsystem(int clawCANId, int flipperCANId) {
        m_claw = new TalonFX(clawCANId);
        m_flipper = new TalonFX(flipperCANId);

        m_mmReq = new MotionMagicVoltage(0);
    }

     /**
     * Set speed of claw motor 
     */
    public void setClawSpeed(double speed){
        m_claw.set(speed);
    }

     /**
     * Set speed of flipper motor 
     */
    public void setFlipperSpeed(double speed){
        m_flipper.set(speed);
    }

    /**
     * Checks if claw motor is stalling/is experiencing a current spike due to successful intake
     *
     * @return True or False
     */
    public boolean holdingBucket(){
        double clawCurrent = m_claw.getStatorCurrent().getValueAsDouble();
        // System.out.println("INTAKE CURRENT: " + intakecur);
        if (clawCurrent > 110){
            return true;
        }
        return false;
    }


    public TalonFX getClaw(){
        return m_claw;
    }

    public TalonFX getFlipper(){
        return m_flipper;
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




    @Override
    public void periodic() {
        holdingBucket();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Flipper voltage: " + m_flipper.getMotorVoltage());
        System.out.println("Intake voltage: " + m_claw.getMotorVoltage());
    
    }
}
