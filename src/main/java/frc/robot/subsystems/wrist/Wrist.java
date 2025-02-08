package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;




public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    
  }

  public Command setWristAngle(double anglePos){
    return runOnce(() -> io.setPose(anglePos));
  }

  public Command moveWrist(double percent){
    return runEnd(() -> io.moveWrist(percent), () -> io.moveWrist(0));
  }

  public TalonFXConfiguration getWristConfigs(){
    return io.getWristConfigs();
  }

  public TalonFX getWristMotor(){
    return io.getWristMotor();
  }

  public MotionMagicVoltage getMMVCont(){
    return io.getmmMMVCont();
  }
  
    
}