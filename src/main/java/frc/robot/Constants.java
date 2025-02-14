package frc.robot;

public class Constants {

  public static final class ElevatorConstants{
    /* CAN IDs */
  public static final int ELEVATOR_LEFT_ID = 21;
  public static final int ELEVATOR_RIGHT_ID = 20;

  /* Elevator Setpoints, Speeds, and Positions  */
  public static final double HOME = 0; //TUNE ME

  public static final double INTAKE_CORAL = 0; //TUNE ME

  public static final double L1 = 3.8; //TUNE ME
  public static final double L2 = 15; //TUNE ME
  public static final double L3 = 30; //TUNE ME
  public static final double L4 = 54; //TUNE ME

  public static final double GROUND_ALGAE = 0; //TUNE ME
  public static final double ALGAE_LOW = 20; //TUNE ME
  public static final double ALGAE_HIGH = 37.5; //TUNE ME

  public static final double NET = 0;



  /* Physical Measurements */
  public static final double ELEVATOR_MIN_HEIGHT = 0;
  public static final double ELEVATOR_MAX_HEIGHT = 55; // (actually ~~65in)
  public static final double ELEVATOR_PULLEY_PITCH_DIAMETER  = 1.504;
  public static final double ELEVATOR_GEAR_RATIO = 8.571;
  }

  public static final class WristConstants{

    public static final int WRIST_ID = 16;
    public static final double WRIST_GEAR_RATIO = 279.27;

    public static final int WRIST_ENCODER_ID = 25;
  
    public static final int currentLimit = 60;
  }

  public static final class AlgaeConstants{
    public static final int ALGAE_ID = 17;
  }

  public static final class ClimberConstants{
    public static final int leftClimberCanId = 15;   
     public static final double motorReduction = 180.0;
    public static final int currentLimit = 40; 
  }

  public static final class CoralMechConstants{
    public static final int CORAL_MECH_ID = 23;
    public static final int CORAL_SENSOR_ID = 26;
  }

}
