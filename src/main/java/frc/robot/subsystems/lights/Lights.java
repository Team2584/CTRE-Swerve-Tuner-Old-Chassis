package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

//Light patterns are on https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf page 14-17

public class Lights extends SubsystemBase {

  Spark Blinkin = new Spark(0);


  public Lights(int BlinkinPort) {
    this.Blinkin = new Spark(BlinkinPort);
    SetColor(Colors.Fire);
  }

  
  public Command SetColor(Color color) {
    return run(()->Blinkin.set(color.GetColorCode()));
  }
}