package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private static CANSparkMax grabberMotor1, grabberMotor2;
    private static DigitalInput laserBreak;
    private boolean grabberValue;
    
    public Grabber() {
        grabberMotor1 = new CANSparkMax(Constants.GRABBER_1_MOTOR_ID, MotorType.kBrushless);
        grabberMotor2 = new CANSparkMax(Constants.GRABBER_2_MOTOR_ID, MotorType.kBrushless);
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
        grabberMotor1.set(-Constants.GRAB_MOTOR_SPEED);
        grabberMotor2.set(-Constants.GRAB_MOTOR_SPEED);
        grabberValue = true;
    }

    // Extends Pneumatic part of the Grabber
    public void Release(double speed1, double speed2) {
        grabberMotor1.set(speed1);
        grabberMotor2.set(speed2);
        grabberValue = false;
    }

    public void stopMotor() {
        grabberMotor1.set(0);
        grabberMotor2.set(0);
    }

    public boolean getLaserBreakValue() {
        return !laserBreak.get();
    }
    
}
