package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/* Interface encapsulating elevator hardware */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double posInches = 0.0;
    public double velMetersPerSecond = 0.0;
    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;
    public double setpointInches = 0.0;
    public double appliedVoltage = 0.0;
    public double[] supplyCurrent = new double[] {}; // {leader, follower}
    public double[] statorCurrent = new double[] {}; // {leader, follower}
    public double acceleration = 0.0;
  }

  /* Updates the set of loggable inputs */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /* Sets the elevator to a height setpoint via motion magic */
  public default void setHeight(double heightInches) {}

  /** Sets the climber to a specified voltage output */
  public default void setVoltage(double volts) {}

  /** Resets the encoder reading of the elevator to a specified position */
  public default void resetHeight(double newHeightInches) {}

  /* Sets the elevators's neutral mode */
  public default void enableBrakeMode(boolean enable) {}

  /* Updates tunable numbers if neccesary */
  public default void updateTunableNumbers() {}

  /** Run music on drive motors for the funny
   *0=Stop, 1=Play, 2=Pause*/
  public default void musicState(String chrpFile) {} //

}