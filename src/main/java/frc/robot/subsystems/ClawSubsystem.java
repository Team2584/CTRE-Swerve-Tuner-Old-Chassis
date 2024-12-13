package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX m_claw;

    public ClawSubsystem(int clawCANId) {
        m_claw = new TalonFX(clawCANId);
    }

     /**
     * Set speed of claw motor 
     */
    public void setClawSpeed(double speed){
        m_claw.set(speed);
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

    public TalonFX getClawMotor(){
        return m_claw;
    }


    @Override
    public void periodic() {
        holdingBucket();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Intake voltage: " + m_claw.getMotorVoltage());
    
    }
}
