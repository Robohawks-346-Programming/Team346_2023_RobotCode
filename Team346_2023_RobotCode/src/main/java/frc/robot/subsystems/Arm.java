package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static CANSparkMax rotationMotor;
    private static DoubleSolenoid brakeSolenoid;
    private static RelativeEncoder rotationEncoder;
    private static SparkMaxPIDController rotationPIDController;

    double armDegreesPerMotorRev;
    
    public Arm() {
        rotationMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);    
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationEncoder = rotationMotor.getEncoder();
        brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.BRAKE_IN_PNEUMATIC_ID, Constants.BRAKE_OUT_PNEUMATIC_ID);

        armDegreesPerMotorRev = 360/Constants.ARM_GEAR_RATIO;
        
        rotationEncoder.setPositionConversionFactor(armDegreesPerMotorRev);
        rotationEncoder.setPosition(Constants.HOME_ARM_ANGLE);
        rotationMotor.setInverted(true);

        rotationPIDController = rotationMotor.getPIDController();
        rotationPIDController.setP(Constants.ARM_P);
        rotationPIDController.setI(Constants.ARM_I);
        rotationPIDController.setD(Constants.ARM_D);

        rotationPIDController.setOutputRange(-Constants.ARM_MOTOR_SPEED_DOWN, Constants.ARM_MOTOR_SPEED_UP);

        rotationMotor.burnFlash();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", getRotationEncoder());
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

    // Checks to see if the position has been reached
    public boolean isAtPosition(double rev) {
        double difference = Math.abs(rotationEncoder.getPosition() - rev);
        return(difference <= Constants.ARM_ANGLE_THRESHOLD);
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
            rotationMotor.set(Constants.ARM_MOTOR_SPEED_UP);
        }

        else if (wantedPosition < currentPosition) {
            rotationMotor.set(-Constants.ARM_MOTOR_SPEED_DOWN);
        }

        else {
            rotationMotor.set(0.0);
        }
    }

    public void moveArmToPosition(double wantedPosition) {
        rotationPIDController.setReference(wantedPosition, ControlType.kPosition);
    }
}
