package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

<<<<<<< HEAD
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private static final TalonFXConfiguration flipperConfig = new TalonFXConfiguration();
    private static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    
    private final TalonFX flipper;
    private final TalonFX intake;

    public ArmSubsystem(int flipperId, int intakeId) {
        flipper = new TalonFX(flipperId);
        intake = new TalonFX(intakeId);

        FeedbackConfigs fdb = flipperConfig.Feedback;
        fdb.SensorToMechanismRatio = 85.33; // 12.8 rotor rotations per mechanism rotation

        MotionMagicConfigs mm = flipperConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(50));


        Slot0Configs slot0 = flipperConfig.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output


        

        StatusCode flipperStatus = flipper.getConfigurator().apply(flipperConfig);
        StatusCode intakeStatus = intake.getConfigurator().apply(intakeConfig);

        if (flipperStatus != StatusCode.OK || intakeStatus != StatusCode.OK) {
            System.err.println("Failed to configure TalonFX motors.");
        }
    
    }

    /**
     * Set intake speed method
     *
     * @return a command
     */
    public Command setIntake() {
        
        if (hasBucket()){
            return runOnce(() -> intake.set(0));
        }
        else{
            // Inline construction of command goes here.
            // Subsystem::RunOnce implicitly requires `this` subsystem.
            armDown();
            return runOnce(() -> intake.set(-0.25));
        }
    }

    public Command setOuttake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> intake.set(0.15));
    }

    public Command killIntake() {

        return runOnce(() -> intake.set(0));
    }

    public Command armDown(){
        System.out.println("ARMPOS: " + flipper.getPosition().getValueAsDouble());
        return runOnce(() -> flipper.setControl(m_mmReq.withPosition(-0.5)));
    }
    public Command armUp(){
        System.out.println("ARMPOS: " + flipper.getPosition().getValueAsDouble());
        return runOnce(() -> flipper.setControl(m_mmReq.withPosition(0)));
    }



    public boolean hasBucket(){
        double intakecur = intake.getStatorCurrent().getValueAsDouble();
        // System.out.println("INTAKE CURRENT: " + intakecur);
        if (intakecur > 110){
            return true;
        }
        return false;
    }

   
    @Override
    public void periodic() {
        hasBucket();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("Flipper voltage: " + flipper.getMotorVoltage());
        System.out.println("Intake voltage: " + intake.getMotorVoltage());
    
    }
}
=======


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
>>>>>>> arm
