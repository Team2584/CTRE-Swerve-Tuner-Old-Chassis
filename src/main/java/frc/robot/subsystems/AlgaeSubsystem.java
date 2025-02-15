package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeSubsystem extends SubsystemBase {

    // Talon Init
    private final TalonFX claw;
    private final TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    public AlgaeSubsystem() {
        claw = new TalonFX(AlgaeConstants.ALGAE_ID);

        claw.getConfigurator().apply(clawConfig);
    }
    
    /**
     * Sets the Algae intake to spin in reverse while being called.
     * Resets speed to zero when not being called.
     */
    public Command outtakeCommand(){
        return runEnd(()->setClawSpeed(0.25),()->setClawSpeed(0));
    }

    /**
     * Sets the Algae intake to spin forward while being called.
     * Resets speed to zero when holding an Algae or when not being called.
     */
    public Command intakeCommand(){
        return runEnd(()->setClawSpeed(-0.25),()->setClawSpeed(0)).until(()->holdingAlgae());
    }

    public Command setSpeed(double speed){
        return run(()->setClawSpeed(speed));
    }


     /**
     * Sets the speed of the Claw motor 
     */
    public void setClawSpeed(double speed){
        claw.set(speed);
    }
    
    /**
     * Checks if claw motor is stalling/is experiencing a current spike due to successful intake
     * @return Whether or not the intake is holding an Algae
     * 
     */
    public boolean holdingAlgae(){
        double clawCurrent = claw.getStatorCurrent().getValueAsDouble();
        // System.out.println("INTAKE CURRENT: " + clawCurrent);
        if (clawCurrent > 100){
            return true;
        }
        return false;
    }

    /**
     * Returns the Wrist Motor
     *
     * @return TalonFX
     */
    public TalonFX getClawMotor(){
        return claw;
    }


    @Override
    public void periodic() {
        holdingAlgae();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("AlgaeIntake voltage: " + claw.getMotorVoltage());
    
    }
}
