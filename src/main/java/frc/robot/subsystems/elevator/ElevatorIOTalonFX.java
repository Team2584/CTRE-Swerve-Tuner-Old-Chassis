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

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX m_leftElevator = new TalonFX(leftElevatorCanId);
  private final TalonFX m_rightElevator = new TalonFX(leftElevatorCanId);

  // Motion Magic control request
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals
  private final StatusSignal<Angle> position = m_leftElevator.getPosition();
  private final StatusSignal<AngularVelocity> velocity = m_leftElevator.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = m_leftElevator.getMotorVoltage();
  private final StatusSignal<Current> current = m_leftElevator.getSupplyCurrent();

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();

    FeedbackConfigs fdb = config.Feedback;
    
    // Configure current limits
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motion Magic configs
    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // Rotations per second
    config.MotionMagic.MotionMagicAcceleration = 10; // Rotations per second squared
    config.MotionMagic.MotionMagicJerk = 50; // Rotations per second cubed

    // Configure Motion Magic and PID gains
    var slot0 = new Slot0Configs();
    slot0.kP = 60; // Adjust based on your elevator's needs
    slot0.kI = 0;
    slot0.kD = 0.5;
    slot0.kV = 0.12; // Roughly 1.2V per RPS
    slot0.kS = 0.25; // Static friction compensation
    config.Slot0 = slot0;

    

    // Set up signal monitoring
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        current
        );
    m_leftElevator.optimizeBusUtilization();

    // Zero the encoder positions
    m_leftElevator.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      position,
      velocity,
      appliedVolts,
      current
        );

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    m_leftElevator.setControl(voltageRequest.withOutput(volts));
  }

  // New method for position control using Motion Magic
  public void setPosition(double positionRotations) {
    m_leftElevator.setControl(motionMagicRequest.withPosition(positionRotations));
    //m_rightElevator.setControl(motionMagicRequest.withPosition(-positionRotations));
  }
}
