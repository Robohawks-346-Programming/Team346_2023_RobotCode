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
    private static CANSparkMax rotationMotor;
    private static DoubleSolenoid armSolenoid1, armSolenoid2, brakeSolenoid;
    private static RelativeEncoder rotationEncoder;

    double armDegreesPerMotorRev;
    
    public Arm() {
        armSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_1_OUT_PNEUMATIC_ID, Constants.ARM_1_IN_PNEUMATIC_ID);
        armSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_2_OUT_PNEUMATIC_ID, Constants.ARM_2_IN_PNEUMATIC_ID);
        rotationMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getEncoder();
        brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.BRAKE_IN_PNEUMATIC_ID, Constants.BRAKE_OUT_PNEUMATIC_ID);

        armDegreesPerMotorRev = 360/Constants.ARM_GEAR_RATIO;
        
        rotationEncoder.setPositionConversionFactor(armDegreesPerMotorRev);
    }

    // Alternative Method to extend Pneumatic part of Arm
    public void extendArmPneumatic1() {
        armSolenoid1.set(Value.kForward);
    }
    

    // Alternative Method to retract Pneumatic part of Arm
    public void retractArmPneumatic1() {
        armSolenoid1.set(Value.kReverse);
    }

    public void extendArmPneumatic2() {
        armSolenoid2.set(Value.kForward);
    }

    public void retractArmPneumatic2() {
        armSolenoid2.set(Value.kReverse);
    }

    // Rotate arm up
    public void RotateUp() {
        rotationMotor.set(Constants.ARM_MOTOR_SPEED);
    }

    // Rotate arm down
    public void RotateDown() {
        rotationMotor.set(-Constants.ARM_MOTOR_SPEED);
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

    public void moveArm(double wantedPosition) {
        double currentPosition = rotationEncoder.getPosition();
        if (wantedPosition > currentPosition) {
            rotationMotor.set(Constants.ARM_MOTOR_SPEED);
        }

        else if (wantedPosition < currentPosition) {
            rotationMotor.set(-Constants.ARM_MOTOR_SPEED);
        }

        else {
            rotationMotor.set(0.0);
        }
    }
}
