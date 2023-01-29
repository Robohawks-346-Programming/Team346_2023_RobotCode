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
    DoubleSolenoid armSolenoid, brakeSolenoid;
    RelativeEncoder rotationEncoder;
    
    public Arm() {
        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_OUT_PNEUMATIC_ID, Constants.ARM_IN_PNEUMATIC_ID);
        rotationMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getEncoder();
        brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.BRAKE_IN_PNEUMATIC_ID, Constants.BRAKE_OUT_PNEUMATIC_ID);
    }

    // Alternative Method to extend Pneumatic part of Arm
    public void extendArmPneumatic() {
        armSolenoid.set(Value.kForward);
    }


    // Alternative Method to retract Pneumatic part of Arm
    public void retractArmPneumatic() {
        armSolenoid.set(Value.kReverse);
    }

    // Extend Pneumatic part of Arm
    public void extendArm() {
        armSolenoid.set(Value.kForward);
    }
    
    // Retract Pneumatic part of Arm
    public void retractArm() {
        armSolenoid.set(Value.kReverse);
    }

    // Rotatate arm up
    public void RotateUp() {
        while(rotationEncoder.getPosition() <= Constants.ARM_REV) {
            rotationMotor.set(Constants.ARM_MOTOR_SPEED);
        }
        rotationMotor.set(0.0);
    }

    // Rotate arm down
    public void RotateDown() {
        while(rotationEncoder.getPosition() <= Constants.ARM_REV) {
            rotationMotor.set(-Constants.ARM_MOTOR_SPEED);
        }
        rotationMotor.set(0.0);
    }

    // Resetting Rotation encoders
    public void resetRotationEncoder() {
        rotationEncoder.setPosition(0.0);
    }

    // Alternative method to rotateUp, does not use while loop
    public void ArmUp(double speed) {
        rotationMotor.set(speed);
    } 

    // Checks to see if the position has been reached
    public boolean isAtPosition(double rev) {
        return (rotationEncoder.getPosition() >= rev);
    }

    // Stops rotation motor once finished
    public void stopRotationMotor() {
        rotationMotor.set(0.0);
    }

    // Stops Rotation with Pneumatic Disc brake
    public void armBrakeOn() {
        brakeSolenoid.set(Value.kForward);
    }

    // Releases Disc brake
    public void armBrakeOff() {
        brakeSolenoid.set(Value.kReverse);
    }
}
