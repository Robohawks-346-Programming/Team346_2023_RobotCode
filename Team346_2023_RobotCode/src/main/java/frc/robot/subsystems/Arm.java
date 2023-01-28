package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax rotationMotor;
    DoubleSolenoid armSolenoid;
    RelativeEncoder rotationEncoder;
    
    public Arm() {
        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GRABBER_OUT_PNEUMATIC_ID, Constants.GRABBER_IN_PNEUMATIC_ID);
        //Need to change to Rev PH
        rotationMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getEncoder();
    }

    public void ExtendArmPneumatic() {
        armSolenoid.set(Value.kForward);
    }

    public void RetractArmPneumatic() {
        armSolenoid.set(Value.kReverse);
    }

    public void RotateArmUp() {
        while(rotationEncoder.getPosition() <= Constants.ARM_REV) {
            rotationMotor.set(Constants.ARM_MOTOR_SPEED);
        }
        rotationMotor.set(0.0);
    }

    public void RotateArmDown() {
        while(rotationEncoder.getPosition() <= Constants.ARM_REV) {
            rotationMotor.set(-Constants.ARM_MOTOR_SPEED);
        }
        rotationMotor.set(0.0);
    }

    public void resetRotationEncoder() {
        rotationEncoder.setPosition(0.0);
    }

    public void ArmUp(double speed) {
        rotationMotor.set(speed);
    } 

    public boolean isAtPosition(double rev) {
        return (rotationEncoder.getPosition() >= rev);
    }

    public void stopRotationMotor() {
        rotationMotor.set(0.0);
    }
}
