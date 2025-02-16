// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.CoralMechConstants;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralMechConstants;


/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class CoralSubsystem extends SubsystemBase {

  /* Hardware */
  private final TalonFX coral;
  private final CANrange coralSensor;

  /* Configs */
  private static final TalonFXConfiguration coralConfigs = new TalonFXConfiguration();
  private static final CANrangeConfiguration coralSensorConfigs = new CANrangeConfiguration();


  public CoralSubsystem() {
    this.coral = new TalonFX(CoralMechConstants.CORAL_MECH_ID);
    this.coralSensor = new CANrange(CoralMechConstants.CORAL_SENSOR_ID);


   
    coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;



    coral.getConfigurator().apply(coralConfigs, 0.25);
    coralSensor.getConfigurator().apply(coralSensorConfigs,0.25);

  }

  /**
   * Shoots the coral out. Coral mechanism will continue to spin while being called. Defaults to not moving.
   */
  public Command shootCoral(){
    return runEnd(() -> setSpeed(0.375), () -> setSpeed(0)).withTimeout(2.5);
  }
  

  
  /**
   * Sets the Coral intake to spin forward while being called.
   * Resets speed to zero when holding a Coral or when not being called.
   */
  public Command intakeCoral(){
    return runEnd(() -> setSpeed(0.375), () -> setSpeed(0)).until(()->hasCoral()).andThen(runEnd(()->setSpeed(0.2),()->setSpeed(0)).withTimeout(0.15));
  }
  
  /**
  * Sets the speed of the coral rollers 
  */
  public void setSpeed(double speed) {
    coral.set(speed);
  }

  public boolean hasCoral(){
    if (coralSensor.getDistance().getValueAsDouble() < 0.1){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean hasCoralDelayed(){
    if (coralSensor.getDistance().getValueAsDouble() < 0.09){
      runOnce(()->new WaitCommand(0.5));
      return true;
    }
    else{
      return false;
    }
  }

 
}
