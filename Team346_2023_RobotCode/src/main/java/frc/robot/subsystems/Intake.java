package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static CANSparkMax intakeMotor;
    private static CANSparkMax rotationMotor;
    private static RelativeEncoder rotationEncoder;
    private static SparkMaxPIDController rotationPIDController;
    private boolean intakeValue;
    private DigitalInput laserBreak;
    
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_MOTOR_ID, MotorType.kBrushless);
        rotationMotor.setIdleMode(IdleMode.kBrake);

        rotationEncoder = rotationMotor.getEncoder();
        rotationPIDController = rotationMotor.getPIDController();

        rotationEncoder.setPosition(0);
        rotationPIDController.setP(Constants.INTAKE_P);
        rotationPIDController.setI(Constants.INTAKE_I);
        rotationPIDController.setD(Constants.INTAKE_D);
        rotationPIDController.setIZone(0);
        rotationPIDController.setFF(0);

        rotationPIDController.setOutputRange(-Constants.INTAKE_MOVE_IN_MOTOR_SPEED, Constants.INTAKE_MOVE_OUT_MOTOR_SPEED);


        intakeValue = false;

        laserBreak = new DigitalInput(Constants.INTAKE_LASER_BREAK_PORT);

        rotationMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Value", intakeValue);
        SmartDashboard.putBoolean("Intake laser", getLaserBreak());
    }
    
    public void moveIntake(double wantedPosition) {
        double currentPosition = rotationEncoder.getPosition();
        double difference = Math.abs(rotationEncoder.getPosition() - wantedPosition);
        while(difference <= Constants.INTAKE_POSITION_THRESHOLD) {
            if (wantedPosition > currentPosition) {
                rotationMotor.set(Constants.INTAKE_MOVE_OUT_MOTOR_SPEED);
            }

            else if (wantedPosition < currentPosition) {
                rotationMotor.set(-Constants.INTAKE_MOVE_IN_MOTOR_SPEED);
            }

            else {
                rotationMotor.set(0.0);
            }
        }
        rotationMotor.set(0.0);
    }

    //Checks if intake is atWantedPosition
    public boolean isAtPosition(double wantedPosition) {
        double difference = Math.abs(rotationEncoder.getPosition() - wantedPosition);
        return(difference <= Constants.INTAKE_POSITION_THRESHOLD);
    }

    public void moveIntakeToPosition(double wantedPosition) {
        rotationPIDController.setReference(wantedPosition, ControlType.kPosition);
    }

    // Runs Intake Motor
    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }
    
    // Stops Intake Motor when finished
    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public boolean getLaserBreak() {
        return !laserBreak.get();
    }

}
