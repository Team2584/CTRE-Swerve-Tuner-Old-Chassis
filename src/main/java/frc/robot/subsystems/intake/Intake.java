package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;



public class Intake extends SubsystemBase {

  private double stallTimerStart = 0;
  private static final double statorCurrentLimit = 80; // Adjust based on testing
  private static final double stallTimer = 0.1; // 100ms, adjust based on testing
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    
  }

  public boolean hasAlgae() {
    if (inputs.intakeStatorCurrent > statorCurrentLimit) {
        if (stallTimerStart == 0) {
            stallTimerStart = Logger.getTimestamp() / 1e6; // Convert to seconds
        }
        return (Logger.getTimestamp()/1e6 - stallTimerStart) > stallTimer;
    }
    stallTimerStart = 0;
    return false;
  }

  public Command moveWrist(double percent){
    return runEnd(() -> io.moveWrist(percent), () -> io.moveWrist(0));
  }


  public Command intakeCommand(double percent) {
    return runEnd(() -> io.moveIntake(percent), () -> io.moveIntake(0.0)).until(()-> hasAlgae());
  }


  public Command outtakeCommand(double percent) {
    return runEnd(() -> io.moveIntake(-1*percent), () -> io.moveIntake(0.0));
  }
    
}