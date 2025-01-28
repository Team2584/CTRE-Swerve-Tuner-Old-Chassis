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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intake = new TalonFX(intakeCanId);

  private final StatusSignal<Angle> intakePosition = intake.getPosition();
  private final StatusSignal<AngularVelocity> intakeVelocity = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeSupplyCurrent = intake.getSupplyCurrent();
  private final StatusSignal<Current> intakeStatorCurrent = intake.getStatorCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;




    tryUntilOk(5, () -> intake.getConfigurator().apply(config, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakePosition,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeStatorCurrent
        );
    intake.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakePosition,
        intakeVelocity,
        intakeAppliedVolts,
        intakeSupplyCurrent,
        intakeStatorCurrent
        );

    inputs.positionRad = Units.rotationsToRadians(intakePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeSupplyCurrent.getValueAsDouble();
    inputs.statorCurrent = intakeStatorCurrent.getValueAsDouble();

  }
  
  @Override
  public void setVoltage(double volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }
}
