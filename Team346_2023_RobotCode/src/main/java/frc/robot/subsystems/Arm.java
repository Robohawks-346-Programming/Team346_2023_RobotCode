package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static CANSparkMax rotationMotor;
    private static DoubleSolenoid armSolenoid1, armSolenoid2, brakeSolenoid;
    private static RelativeEncoder rotationEncoder;
    private boolean arm1Value, arm2Value;

    double armDegreesPerMotorRev;
    
    public Arm() {
        armSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_1_OUT_PNEUMATIC_ID, Constants.ARM_1_IN_PNEUMATIC_ID);
        armSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_2_OUT_PNEUMATIC_ID, Constants.ARM_2_IN_PNEUMATIC_ID);
        rotationMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);      //Test ID
        rotationEncoder = rotationMotor.getEncoder();
        brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.BRAKE_IN_PNEUMATIC_ID, Constants.BRAKE_OUT_PNEUMATIC_ID);

        armDegreesPerMotorRev = 360/Constants.ARM_GEAR_RATIO;
        
        rotationEncoder.setPositionConversionFactor(armDegreesPerMotorRev);

        arm1Value = false;
        arm2Value = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", getRotationEncoder());
        SmartDashboard.putBoolean("Arm 1 value", arm1Value);
        SmartDashboard.putBoolean("Arm 2 Value", arm2Value);
    }

    // Alternative Method to extend Pneumatic part of Arm
    public void extendArmPneumatic1() {
        armSolenoid1.set(Value.kForward);
        arm1Value = true;
    }
    

    // Alternative Method to retract Pneumatic part of Arm
    public void retractArmPneumatic1() {
        armSolenoid1.set(Value.kReverse);
        arm1Value = false;
    }


    public void extendArmPneumatic2() {
        armSolenoid2.set(Value.kForward);
        arm2Value = true;
    }

    public void retractArmPneumatic2() {
        armSolenoid2.set(Value.kReverse);
        arm2Value = false;
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

    public double getRotationEncoder() {
        return rotationEncoder.getPosition();
    }

    public void setRotationEncoder() {
        rotationEncoder.setPosition(Constants.HOME_ARM_ANGLE);
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
