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
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commons.TunableDashboardNumber;

public class ElevatorIOTalonFX implements ElevatorIO {

  /* Hardware */
  private final TalonFX leader;
  private final TalonFX follower;

  /* Configurators */
  private TalonFXConfiguration leaderConfiguration;

  /* Configs */
  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs leaderMotorConfigs;
  private final MotorOutputConfigs followerMotorConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;

  /* Gains */
  TunableDashboardNumber kS = new TunableDashboardNumber("Elevator/kS", 0);
  TunableDashboardNumber kG = new TunableDashboardNumber("Elevator/kA", 0);
  TunableDashboardNumber kV = new TunableDashboardNumber("Elevator/kV", 0);
  TunableDashboardNumber kP = new TunableDashboardNumber("Elevator/kP", 1.0);
  TunableDashboardNumber kI = new TunableDashboardNumber("Elevator/kI", 0.0);
  TunableDashboardNumber kD = new TunableDashboardNumber("Elevator/kD", 0.5);

  TunableDashboardNumber motionAcceleration = new TunableDashboardNumber("Elevator/MotionAcceleration", 10);
  TunableDashboardNumber motionCruiseVelocity = new TunableDashboardNumber("Elevator/MotionCruiseVelocity", 0.5);
  TunableDashboardNumber motionJerk = new TunableDashboardNumber("Elevator/MotionJerk", 50);

  /* Status Signals */
  private StatusSignal<Current> supplyLeft;
  private StatusSignal<Current> supplyRight;
  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;

  public ElevatorIOTalonFX() {
    /* Instantiate motors and configurators */
    this.leader = new TalonFX(ELEVATOR_LEFT_ID);
    this.follower = new TalonFX(ELEVATOR_RIGHT_ID);

    this.leaderConfiguration = new TalonFXConfiguration();

    this.leader.setPosition(0);

    /* Create Configs */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 120.0;
    currentLimitsConfigs.SupplyCurrentLimit = 120;


    leaderMotorConfigs = new MotorOutputConfigs();
    leaderMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    followerMotorConfigs = new MotorOutputConfigs();
    followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;


    slot0Configs = leaderConfiguration.Slot0; //PID Gains
    slot0Configs.kP = kP.get(); // Adjust based on your elevator's needs
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kV = kV.get(); // Roughly 1.2V per RPS
    slot0Configs.kG = kG.get(); // Adds constant value to hold elevator up
    slot0Configs.kS = kS.get(); // Static friction compensation

    motionMagicConfigs = leaderConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get(); // Rotations per second squared
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();// Rotations per second
    motionMagicConfigs.MotionMagicJerk = motionJerk.get(); // Rotations per second cubed

    /* Apply Configs */
    leader.getConfigurator().apply(leaderConfiguration);


    // Set up signal monitoring
    supplyLeft = leader.getSupplyCurrent();
    supplyRight = follower.getSupplyCurrent();
    closedLoopReferenceSlope = leader.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, supplyLeft, supplyRight, closedLoopReferenceSlope);

    follower.setControl(new Follower(ELEVATOR_LEFT_ID, false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(supplyLeft, supplyRight, closedLoopReferenceSlope);

    inputs.posInches = getHeight();
    inputs.velMetersPerSecond = getVelocity();
    inputs.motionMagicVelocityTarget = rotationsToInches(leader.getClosedLoopReferenceSlope().getValue());
    inputs.motionMagicPositionTarget = rotationsToInches(leader.getClosedLoopReference().getValue());
    inputs.appliedVoltage = leader.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = new double[] {supplyLeft.getValueAsDouble(), supplyRight.getValueAsDouble()};
    inputs.statorCurrent = new double[] {leader.getStatorCurrent().getValueAsDouble(), follower.getStatorCurrent().getValueAsDouble()};
    inputs.setpointInches = setpoint;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    if (currentTime - prevReferenceSlopeTimestamp > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope)
              / (currentTime - prevReferenceSlopeTimestamp);
    }

    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;
  }

  @Override
  public void setHeight(double heightInches) {
    if (!DriverStation.isEnabled()) {
      return;
    }
    setpoint = heightInches;
    leader.setControl(new MotionMagicVoltage(inchesToRotations(heightInches)));
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void resetHeight(double newHeightInches) {
    leader.setPosition(inchesToRotations(newHeightInches));
  }

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kG.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionAcceleration.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
      slot0Configs.kS = kS.get();
      slot0Configs.kG = kG.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      leader.getConfigurator().apply(leaderConfiguration);
    }
  }

  private double getHeight() {
    return rotationsToInches(leader.getPosition().getValueAsDouble());
  }

  private double getVelocity() {
    return rotationsToInches(leader.getVelocity().getValueAsDouble());
  }

  private double inchesToRotations(double heightInches) {
    return (heightInches / (Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)) * ELEVATOR_GEAR_RATIO;
}

  private double rotationsToInches(double rotations) {
    return rotations * (Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER) / ELEVATOR_GEAR_RATIO;
  }
}
