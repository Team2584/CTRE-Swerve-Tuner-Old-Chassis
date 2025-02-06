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

package frc.robot.subsystems.coral;

import static frc.robot.subsystems.coral.CoralConstants.*;
import static frc.robot.util.PhoenixUtil.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;


/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class CoralIOTalonFX implements CoralIO {

  /* Hardware */
  private final TalonFX coral;

  /* Configs */
  private static final TalonFXConfiguration coralConfigs = new TalonFXConfiguration();


  /* Status Signals */
  private final StatusSignal<AngularVelocity> coralVelocity;
  private final StatusSignal<Voltage> coralAppliedVolts;


  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public CoralIOTalonFX() {
    this.coral = new TalonFX(CORAL_MECH_ID);
   
    coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Initialize Status Signals */
    coralVelocity = coral.getVelocity();
    coralAppliedVolts = coral.getMotorVoltage();



   
    coral.getConfigurator().apply(coralConfigs, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        coralVelocity,
        coralAppliedVolts
        );
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      coralVelocity,
      coralAppliedVolts
    );

    inputs.coralVelocity = Units.rotationsToRadians(coralVelocity.getValueAsDouble());
    inputs.coralAppliedVolts = coralAppliedVolts.getValueAsDouble();
  }
  
  @Override
  public void setVoltage(double volts) {
    coral.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setSpeed(double speed) {
    coral.set(speed);
  }

 
}
