package frc.robot.subsystems.wrist;
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

  /**Run wrist at specified percentage of max voltage. */ 
  public default void moveWrist(double percent){}
  
}