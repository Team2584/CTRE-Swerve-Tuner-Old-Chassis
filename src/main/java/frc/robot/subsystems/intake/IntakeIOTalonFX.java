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
import frc.robot.util.TunableDashboardNumber;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {

  /* Hardware */
  private final TalonFX intake;

  /* Configs */
  private static final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
  

  /* Status Signals */
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Current> intakeStatorCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    this.intake = new TalonFX(INTAKE_ID);
    
    /* Initialize Status Signals */
    intakeVelocity = intake.getVelocity();
    intakeAppliedVolts = intake.getMotorVoltage();
    intakeSupplyCurrent = intake.getSupplyCurrent();
    intakeStatorCurrent = intake.getStatorCurrent();
   

    /* Apply Configs */
    intake.getConfigurator().apply(intakeConfigs, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeStatorCurrent
        );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      intakeVelocity,
      intakeAppliedVolts,
      intakeSupplyCurrent,
      intakeStatorCurrent
    );

    inputs.intakeVelocity = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
  }
  
  @Override
  public void setVoltage(double volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }


  @Override
  public void moveIntake(double percent){
    intake.setControl(voltageRequest.withOutput((percent/100)*12));
  }
  
}
