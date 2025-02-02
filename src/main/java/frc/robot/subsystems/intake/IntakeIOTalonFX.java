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

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.commons.TunableDashboardNumber;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {

  /* Hardware */
  private final TalonFX intake;
  private final TalonFX wrist;

  /* Configs */
  private static final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration wristConfigs = new TalonFXConfiguration();
  private final MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVoltage motionProfileReq;
  private Slot0Configs slot0Configs;


  private double setpoint;

  /* Tunable Gains */
  TunableDashboardNumber kS = new TunableDashboardNumber("Wrist/kS", 0.25);
  TunableDashboardNumber kV = new TunableDashboardNumber("Wrist/kV", 0.12);
  TunableDashboardNumber kA = new TunableDashboardNumber("Wrist/kA", 0.01);
  TunableDashboardNumber kP = new TunableDashboardNumber("Wrist/kP", 10);
  TunableDashboardNumber kI = new TunableDashboardNumber("Wrist/kI", 0.0);
  TunableDashboardNumber kD = new TunableDashboardNumber("Wrist/kD", 0.05);

  TunableDashboardNumber motionCruiseVelocity = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 0.5);
  TunableDashboardNumber motionAcceleration = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 10);
  TunableDashboardNumber motionJerk = new TunableDashboardNumber("Wrist/MotionCruiseVelocity", 50);

  /* Status Signals */
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristSupplyCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    this.intake = new TalonFX(INTAKE_ID);
    this.wrist = new TalonFX(WRIST_ID);

    motionProfileReq = new MotionMagicVoltage(0); // This is the motion profile, all setControl must target position based on this (m_)

    FeedbackConfigs fdb = wristConfigs.Feedback;
    fdb.SensorToMechanismRatio = WRIST_GEAR_RATIO;

    /* Create Configs */
    wristConfigs.CurrentLimits.SupplyCurrentLimit = 120;
    wristConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    slot0Configs = wristConfigs.Slot0; //PID Gains
    slot0Configs.kP = kP.get(); // Adjust based on your elevator's needs
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get(); // Static friction compensation
    slot0Configs.kV = kV.get(); // Roughly 1.2V per RPS
    slot0Configs.kA = kA.get(); // Roughly 1.2V per RPS


    /* Initialize Status Signals */
    intakeVelocity = intake.getVelocity();
    intakeAppliedVolts = intake.getMotorVoltage();
    intakeSupplyCurrent = intake.getSupplyCurrent();
    intakeStatorCurrent = intake.getStatorCurrent();
    wristPosition = wrist.getPosition();
    wristSupplyCurrent = wrist.getSupplyCurrent();
    wristAppliedVolts = wrist.getMotorVoltage();
    wristVelocity = wrist.getVelocity();


    //Add MotionMagic to Configs
    motionMagicConfigs = wristConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get(); // Rotations per second
    motionMagicConfigs.MotionMagicAcceleration=  motionAcceleration.get(); // Rotations per second^2
    motionMagicConfigs.MotionMagicJerk =  motionJerk.get(); // Rotations per second ^3

    /* Apply Configs */
    intake.getConfigurator().apply(intakeConfigs, 0.25);
    wrist.getConfigurator().apply(wristConfigs, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeStatorCurrent,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristSupplyCurrent
        );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      intakeVelocity,
      intakeAppliedVolts,
      intakeSupplyCurrent,
      intakeStatorCurrent,
      wristPosition,
      wristVelocity,
      wristAppliedVolts,
      wristSupplyCurrent
    );

    inputs.intakeVelocity = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
    inputs.wristPosition = Units.rotationsToRadians(wristPosition.getValueAsDouble());
    inputs.wristVelocity = Units.rotationsToRadians(wristVelocity.getValueAsDouble());
    inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = wristSupplyCurrent.getValueAsDouble();

  }
  
  @Override
  public void setVoltage(double volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void moveIntake(double percent){
    intake.setControl(voltageRequest.withOutput((percent/100)*12));
  }
  @Override
  public void moveWrist(double percentSpeed){
    wrist.set(percentSpeed);
  }
}
