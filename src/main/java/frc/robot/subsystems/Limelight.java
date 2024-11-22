package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.Swerve.generated.TunerConstants;
// import frc.util.InterpolatingTreeMap;

public class Limelight extends SubsystemBase {
    String L_name;
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    NetworkTableEntry tid;
    double aprilTag;
    double Kp = 0.02; // Proportional control constant
    double min_command = 0.15; // Minimum amount to slightly move
    double steering_adjust;
    double limelightMountAngleDegrees = 24.0; 
    double limelightLensHeightInches = 15.0 + (3/16); 
    double heightOfGoal = 58.33;
    double limelight_kP = 0.015;
    double mapOffset = 0;
    
    public Limelight(String n_table){
        L_name = n_table;
        table = NetworkTableInstance.getDefault().getTable(n_table);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        table.getEntry("pipeline").setNumber(0);
        //  LimelightHelpers.getTX(L_name);
    }



    public double getTX(){
        return tx.getDouble(0.0);
    }

    public double getTV(){
        return tv.getDouble(0.0);
    }

    public double getTY(){
        return ty.getDouble(0.0);
    }

    public double getArea(){
       return ta.getDouble(0.0);
    }

    public double getID(){
        return tid.getDouble(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber(L_name + " XPos", getTX());
        SmartDashboard.putNumber(L_name + " YPos", getTY());
        SmartDashboard.putNumber(L_name + " Area", getArea());
        SmartDashboard.putNumber(L_name + " April Tag", getID());
    }

    
 
}