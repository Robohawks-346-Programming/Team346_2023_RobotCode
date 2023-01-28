package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private static Spark LED1 = null;
    private static Spark LED2 = null;

    public void cone() {
        LED1.set(0.69); //Yellow
    }

    public void cube() {
        LED1.set(0.91); //Purple
    }

    public void singleSubstation() {
        LED2.set(0.61); //Red
    }

    public void doubleSubstation() {
        LED2.set(0.77); //Green
    }
}
