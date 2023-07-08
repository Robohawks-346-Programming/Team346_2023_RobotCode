package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase{
    private static Spark LED1;

    public LED() {
        LED1 = new Spark(Constants.LED_1_PWM_PORT);
    }

    // Changes LED to Green for a Double Substation

    public void red() {
        LED1.set(0.61); //red
    }

    public void green() {
        LED1.set(0.77); //Green
    }

    public void light() {
        if (RobotContainer.grabber.getLaserBreakValue() || RobotContainer.intake.getLaserBreak()) {
            green();
        }
        else {
            red();
        }
    }
}
