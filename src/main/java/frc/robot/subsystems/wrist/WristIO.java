package frc.robot.subsystems.wrist;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double wristPosition = 0.0;
    public double wristVelocity = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  
  /** Update the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**Move wrist to specified setpoint using a Motion Profile and PID. */ 
  public default void setPose(double pose) {}

  /**Run wrist at specified percentage of max voltage. */ 
  public default void moveWrist(double percent){}

  /* Returns the motor for the wrist subsystem, Correct override is in WristIOTalonFX*/
  public default TalonFX getWristMotor() {return new TalonFX(22);}

  /* Returns the motor configuration for the wrist motor, Correct override is in WristIOTalonFX*/
  public default TalonFXConfiguration getWristConfigs(){return new TalonFXConfiguration();}

  public default MotionMagicVoltage getmmMMVCont(){return new MotionMagicVoltage(0);}

  /* Updates tunable numbers if neccesary */
  public default void updateTunableNumbers() {}
  
  
}