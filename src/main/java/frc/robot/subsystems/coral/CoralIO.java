package frc.robot.subsystems.coral;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  public static class CoralIOInputs {
    public double coralVelocity = 0.0;
    public double coralAppliedVolts = 0.0;
  }
   

  /** Update the set of loggable inputs. */
  public default void updateInputs(CoralIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  // Sets speed of coral mechanism's rollers
  public default void setSpeed(double speed){}
  
}