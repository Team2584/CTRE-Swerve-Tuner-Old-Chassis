package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
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


        FeedbackConfigs fdb = flipperConfig.Feedback;
        fdb.SensorToMechanismRatio = 85.33; // 12.8 rotor rotations per mechanism rotation

        MotionMagicConfigs mm = flipperConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(50));


        /**
         * Configure Motor Constants for Flipper Motor Using Motion Magic
         */

        Slot0Configs slot0 = flipperConfig.Slot0; 
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output


        StatusCode intakeStatus = m_claw.getConfigurator().apply(intakeConfig); // Configure default Motor Constants for claw motor without Motion Magic
    

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
