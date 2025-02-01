package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  // Create elevator simulation
  private final ElevatorSim sim = new ElevatorSim(
      // 2 Kraken X60 motors
      DCMotor.getKrakenX60(2),
      ELEVATOR_GEAR_RATIO,
      // Carriage mass (kg)
      10.0,
      // Pulley radius (m)
      Units.inchesToMeters(ELEVATOR_PULLEY_PITCH_DIAMETER / 2.0),
      // Min height (m)
      Units.inchesToMeters(ELEVATOR_MIN_HEIGHT),
      // Max height (m)
      Units.inchesToMeters(ELEVATOR_MAX_HEIGHT),
      // Simulate gravity
      true,
      // Starting position (m)
      0.0);

  // Motion Magic simulation
  private final PIDController pidController = new PIDController(10.0, 0.0, 0.5);
  private double targetPositionInches = 0.0;
  private double appliedVolts = 0.0;
  private double setpoint = 0.0;

  // Simulated motion profile
  private double cruiseVelocityInchesPerSec = 50.0;
  private double accelerationInchesPerSecSq = 100.0;
  private double lastTimeSeconds = 0.0;

  public ElevatorIOSim() {
    pidController.setTolerance(0.5); // 0.5 inch position tolerance
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Calculate time delta
    double currentTime = Timer.getTimestamp();
    double dt = currentTime - lastTimeSeconds;
    lastTimeSeconds = currentTime;

    // Calculate motion profile
    double positionError = targetPositionInches - Units.metersToInches(sim.getPositionMeters());
    double velocityInchesPerSec = sim.getVelocityMetersPerSecond() * 39.37; // Convert to inches/sec

    // Simple motion profile implementation
    double targetVelocity = 0.0;
    if (Math.abs(positionError) > 0.1) { // Dead band
      targetVelocity = Math.signum(positionError) * cruiseVelocityInchesPerSec;
      // Apply acceleration limit
      double maxDeltaVelocity = accelerationInchesPerSecSq * dt;
      targetVelocity = MathUtil.clamp(
          targetVelocity,
          velocityInchesPerSec - maxDeltaVelocity,
          velocityInchesPerSec + maxDeltaVelocity);
    }

    // Calculate voltage with PID and feedforward
    double pidVoltage = pidController.calculate(velocityInchesPerSec, targetVelocity);
    double gravityVoltage = 0.12; // Voltage to counteract gravity
    appliedVolts = MathUtil.clamp(pidVoltage + gravityVoltage, -12.0, 12.0);

    // Update simulation
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    // Update inputs
    inputs.posInches = Units.metersToInches(sim.getPositionMeters());
    inputs.velMetersPerSecond = sim.getVelocityMetersPerSecond();
    inputs.motionMagicVelocityTarget = targetVelocity;
    inputs.motionMagicPositionTarget = targetPositionInches;
    inputs.appliedVoltage = appliedVolts;
    inputs.setpointInches = setpoint;
    inputs.supplyCurrent = new double[] {sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};
    inputs.statorCurrent = new double[] {sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};
    inputs.acceleration = (targetVelocity - velocityInchesPerSec) / dt;
  }

  @Override
  public void setHeight(double heightInches) {
    targetPositionInches = MathUtil.clamp(
        heightInches,
        ELEVATOR_MIN_HEIGHT,
        ELEVATOR_MAX_HEIGHT);
    setpoint = heightInches;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void resetHeight(double heightInches) {
    sim.setState(Units.inchesToMeters(heightInches), 0.0);
    targetPositionInches = heightInches;
    setpoint = heightInches;
  }

  @Override
  public void updateTunableNumbers() {
    // In simulation, we could update PID gains and motion profile parameters here
  }
}