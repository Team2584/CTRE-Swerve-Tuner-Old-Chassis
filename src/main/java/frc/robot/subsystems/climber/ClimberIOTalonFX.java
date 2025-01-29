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

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftClimber = new TalonFX(leftClimberCanId);
  private final TalonFX rightClimber = new TalonFX(rightClimberCanId);

  private final StatusSignal<Angle> leftPosition = leftClimber.getPosition();
  private final StatusSignal<AngularVelocity> leftVelocity = leftClimber.getVelocity();
  private final StatusSignal<Voltage> leftAppliedVolts = leftClimber.getMotorVoltage();
  private final StatusSignal<Current> leftCurrent = leftClimber.getSupplyCurrent();

  private final StatusSignal<Angle> rightPosition = rightClimber.getPosition();
  private final StatusSignal<AngularVelocity> rightVelocity = rightClimber.getVelocity();
  private final StatusSignal<Voltage> rightAppliedVolts = rightClimber.getMotorVoltage();
  private final StatusSignal<Current> rightCurrent = rightClimber.getSupplyCurrent();


  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ClimberIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> leftClimber.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightClimber.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);
    leftClimber.optimizeBusUtilization();
    rightClimber.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent
        );

    inputs.positionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.appliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.currentAmps = leftCurrent.getValueAsDouble();

  }

  @Override
  public void setVoltage(double volts) {
    leftClimber.setControl(voltageRequest.withOutput(volts));
    rightClimber.setControl(voltageRequest.withOutput(-volts));
  }
}
