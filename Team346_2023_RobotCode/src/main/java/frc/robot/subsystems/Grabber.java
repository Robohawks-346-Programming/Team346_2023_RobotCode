package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private static DoubleSolenoid grabberSolenoid;
    private static DigitalInput laserBreak;
    private boolean grabberValue;
    
    public Grabber() {
        grabberSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, Constants.GRABBER_OUT_PNEUMATIC_ID, Constants.GRABBER_IN_PNEUMATIC_ID);
        laserBreak = new DigitalInput(Constants.GRABBER_LASER_BREAK_PORT);

        grabberValue = false;
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Laser Break", getLaserBreakValue());
        SmartDashboard.putBoolean("Grabber value", grabberValue);
    }
    // Retracts Pneumatic part of the Grabber
    public void Grab() {
        grabberSolenoid.set(Value.kForward);
        grabberValue = true;
    }

    // Extends Pneumatic part of the Grabber
    public void Release() {
        grabberSolenoid.set(Value.kReverse);
        grabberValue = false;
    }

    public boolean getLaserBreakValue() {
        return laserBreak.get();
    }
    
}
