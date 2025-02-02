package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocity = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeStatorCurrent = 0.0;

    public double wristPosition = 0.0;
    public double wristVelocity = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }
   

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run intake at specified percentage of max voltage. */  
  public default void moveIntake(double percent){}

  /**Run wrist at specified percentage of max voltage. */ 
  public default void moveWrist(double percent){}
  
}