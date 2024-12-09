package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {


    // Talon Init
    private final TalonFX m_claw;
    private final TalonFX m_flip;

    
    public ArmSubsystem(int clawCAN, int flipCAN){
        m_claw = new TalonFX(clawCAN);
        m_flip = new TalonFX(flipCAN);

    }

    public void setClawSpeed(double speed){

    }

    public void setFlipSpeed(double speed){

    }
}
