package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

import static edu.wpi.first.units.Units.Second;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  final double FIRST_STAGE_MIN_HEIGHT = 0;
  final double FIRST_STAGE_MAX_HEIGHT = 28.625*0.0254;
  final double SECOND_STAGE_MIN_HEIGHT = 0;
  final double SECOND_STAGE_MAX_HEIGHT = 53.375*0.0254;


  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double firstStageHeight = Math.max(FIRST_STAGE_MIN_HEIGHT,Math.min(FIRST_STAGE_MAX_HEIGHT,0.68*Math.sin((1/1.6)*Timer.getTimestamp()) + 0.68));

    double secondStageHeight = Math.max(SECOND_STAGE_MIN_HEIGHT,Math.min(SECOND_STAGE_MAX_HEIGHT,0.68*Math.sin((1/1.6)*Timer.getTimestamp()) + 0.68));

    // double secondStageHeight = totalHeightfromMotor - firstStageHeight;


    // Log the 3D Translation of the 1st Elevator stage for AdvantageScope
    Logger.recordOutput("Elevator/1st Stage zeroed", new Pose3d[] {new Pose3d()});
    Logger.recordOutput("Elevator/2nd Stage zeroed", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
      "Elevator/1st stage Final'",
      new Pose3d[] {
        new Pose3d(
          0, 0.0, firstStageHeight, new Rotation3d(0,0,0)
        )
      });

    
      // Log the 3D Translation of the 2nd Elevator stage for AdvantageScope
      Logger.recordOutput(
      "Elevator/2nd Stage Final'",
      new Pose3d[] {
        new Pose3d(
          0, 0, secondStageHeight, new Rotation3d(0,0,0)
        )
      });

  
  }



  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  
}