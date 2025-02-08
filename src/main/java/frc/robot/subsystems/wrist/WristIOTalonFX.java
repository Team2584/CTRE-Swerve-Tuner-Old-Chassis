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

package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.TunableDashboardNumber;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class WristIOTalonFX implements WristIO {

  /* Hardware */
  private final TalonFX wrist;
  // private final CANdi wristEncoder;

  /* Configs */
  private static final TalonFXConfiguration wristConfigs = new TalonFXConfiguration();
  // private static final CANdiConfiguration wristEncoderConfigs = new CANdiConfiguration();
  // private final MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVoltage motionProfileReq;
  // private final VoltageOut voltageRequest;
  private Slot0Configs slot0Configs;


  private double setpoint;

  /* Tunable Gains */
  TunableDashboardNumber kS = new TunableDashboardNumber("Wrist/kS", 0.3);
  TunableDashboardNumber kV = new TunableDashboardNumber("Wrist/kV", 0.1);
  TunableDashboardNumber kA = new TunableDashboardNumber("Wrist/kA", 0.01);
  TunableDashboardNumber kG = new TunableDashboardNumber("Wrist/kG", 0.0);
  TunableDashboardNumber kP = new TunableDashboardNumber("Wrist/kP", 33);
  TunableDashboardNumber kI = new TunableDashboardNumber("Wrist/kI", 0.0);
  TunableDashboardNumber kD = new TunableDashboardNumber("Wrist/kD", 0.8);

  TunableDashboardNumber motionCruiseVelocity = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 5);
  TunableDashboardNumber motionAcceleration = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 8);
  TunableDashboardNumber motionJerk = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 40);

  /* Status Signals */
  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristSupplyCurrent;


  public WristIOTalonFX() {
    this.wrist = new TalonFX(WRIST_ID);
    // this.wristEncoder = new CANdi(WRIST_ENCODER_ID);

    motionProfileReq = new MotionMagicVoltage(0); // This is the motion profile, all setControl must target position based on this (m_)
    // voltageRequest = new VoltageOut(0);

    // wristEncoderConfigs.PWM1.AbsoluteSensorDiscontinuityPoint = 1;
    // wristEncoderConfigs.PWM1.AbsoluteSensorOffset = -0.9335;
    // wristEncoder.getConfigurator().apply(wristEncoderConfigs);


    // FeedbackConfigs fdb = wristConfigs.Feedback;
    // // fdb.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
    // // fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
    // fdb.SensorToMechanismRatio = WRIST_GEAR_RATIO;

    this.wrist.setPosition(-0.25);
    /* Create Configs */
    // wristConfigs.CurrentLimits.SupplyCurrentLimit = 120;
    // wristConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // slot0Configs = wristConfigs.Slot0; //PID Gains
    // slot0Configs.kP = kP.get(); // Adjust based on your needs
    // slot0Configs.kI = kI.get();
    // slot0Configs.kD = kD.get();
    // slot0Configs.kG = kG.get();
    // slot0Configs.kS = kS.get(); // Static friction compensation
    // slot0Configs.kV = kV.get(); // Roughly 1.2V per RPS
    // slot0Configs.kA = kA.get(); // Roughly 1.2V per RPS
    // slot0Configs.kG = kG.get(); // Roughly 1.2V per RPS
    // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;


    /* Initialize Status Signals */
    wristPosition = wrist.getPosition();
    wristSupplyCurrent = wrist.getSupplyCurrent();
    wristAppliedVolts = wrist.getMotorVoltage();
    wristVelocity = wrist.getVelocity();


    //Add MotionMagic to Configs
    // motionMagicConfigs = wristConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get(); // Rotations per second
    // motionMagicConfigs.MotionMagicAcceleration=  motionAcceleration.get(); // Rotations per second^2
    // motionMagicConfigs.MotionMagicJerk =  motionJerk.get(); // Rotations per second ^3

    

    /* Apply Configs */
    // wrist.getConfigurator().apply(wristConfigs, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristSupplyCurrent
        );
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      wristPosition,
      wristVelocity,
      wristAppliedVolts,
      wristSupplyCurrent
    );

    inputs.wristPosition = Units.rotationsToDegrees(wristPosition.getValueAsDouble());
    inputs.wristVelocity = Units.rotationsToDegrees(wristVelocity.getValueAsDouble());
    inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = wristSupplyCurrent.getValueAsDouble();

  }
  
  // @Override
  // public void setVoltage(double volts) {
  //   wrist.setControl(voltageRequest.withOutput(volts));
  // }

  @Override
  public TalonFX getWristMotor(){
    return wrist;
  }

  @Override
  public TalonFXConfiguration getWristConfigs(){
    return wristConfigs;
  }

  @Override
  public MotionMagicVoltage getmmMMVCont(){
    return motionProfileReq;
  }
  

  @Override
  public void setPose(double pose) {
    if (!DriverStation.isEnabled()) {
      return;
    }
    setpoint = pose;
    wrist.setControl(motionProfileReq.withPosition(Units.degreesToRotations(pose)));
  }

  // @Override
  // public void moveWrist(double percentSpeed){
  //   wrist.setControl(voltageRequest.withOutput((percentSpeed/100)*12));
  // }
  
}
