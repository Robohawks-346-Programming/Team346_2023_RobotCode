package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private static Spark LED = null;

    public void cone() {
        LED.set(0.69);
    }

    public void cube() {
        LED.set(0.91);
    }
}
