package frc.robot.subsystems.lights;

public class Color {
    private double colorCode;

    public Color(double colorCode){
        this.colorCode = colorCode;
    }

    public double GetColorCode(){
        return colorCode;
    }
}
