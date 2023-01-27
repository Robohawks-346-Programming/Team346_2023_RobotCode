package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    DoubleSolenoid grabberSolenoid;
    
    public Grabber() {
        grabberSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.GRABBER_OUT_PNEUMATIC_ID, Constants.GRABBER_IN_PNEUMATIC_ID);
    }

    public void Grab() {
        grabberSolenoid.set(Value.kForward);
    }

    public void Release() {
        grabberSolenoid.set(Value.kReverse);
    }
}
