package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends SubsystemBase {

  Spark Blinkin = new Spark(0);

  HashMap<String, Double> Colors = new HashMap<String, Double>();

  public Lights(int BlinkinPort) {
    this.Blinkin = new Spark(BlinkinPort);
    Colors.put("Fire", -.57);
    Colors.put("Green", .71);
  }

  public Command SetColor(String color) {
    return run(()->Blinkin.set(Colors.get(color)));
  }
}