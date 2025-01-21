package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    // Talon Init
    private final TalonFX[] m_climber;

    public Climber(int climberCANId1, int climberCANId2) {
        m_climber = new TalonFX[]{new TalonFX(climberCANId1), new TalonFX(climberCANId2)};

        for (TalonFX currentMotor : m_climber) {
            currentMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

     /**
     * Set speed of climber motor 
     */
    public void setClimberSpeed(double speed){
        m_climber[0].set(speed);
        m_climber[1].set(-speed);
    }

    /**
     * Checks if climber motor is stalling/is experiencing a current spike due to successful intake
     *
     * @return True or False
     */
    public boolean holdingCage(){
        double climberCurrent = m_climber[0].getStatorCurrent().getValueAsDouble();
        // System.out.println("INTAKE CURRENT: " + intakecur);
        SmartDashboard.putNumber("Climber current", climberCurrent);
        if (climberCurrent > 110){
            return true;
        }
        return false;
    }

    public TalonFX[] getclimberMotors(){
        return m_climber;
    }


    @Override
    public void periodic() {
        holdingCage();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Intake voltage: " + m_climber[0].getMotorVoltage() + "and " + m_climber[1].getMotorVoltage());
    }
}
