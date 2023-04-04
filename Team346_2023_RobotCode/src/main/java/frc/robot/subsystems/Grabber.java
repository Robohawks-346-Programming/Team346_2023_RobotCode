package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private static CANSparkMax grabberMotor1, grabberMotor2;
    private static DigitalInput laserBreak;
    
    public Grabber() {
        grabberMotor1 = new CANSparkMax(Constants.GRABBER_1_MOTOR_ID, MotorType.kBrushless);
        grabberMotor2 = new CANSparkMax(Constants.GRABBER_2_MOTOR_ID, MotorType.kBrushless);
        grabberMotor1.setIdleMode(IdleMode.kBrake);
        grabberMotor2.setIdleMode(IdleMode.kBrake);
        laserBreak = new DigitalInput(Constants.GRABBER_LASER_BREAK_PORT);

    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Grabber laser", getLaserBreakValue());
    }
    public void Grab() {
        grabberMotor1.set(-Constants.GRAB_MOTOR_SPEED);
        grabberMotor2.set(-Constants.GRAB_MOTOR_SPEED);
    }

    public void Release(double speed1, double speed2) {
        grabberMotor1.set(speed1);
        grabberMotor2.set(speed2);
    }

    public void stopMotor() {
        grabberMotor1.set(0);
        grabberMotor2.set(0);
    }

    public boolean getLaserBreakValue() {
        return !laserBreak.get();
    }
    
}
