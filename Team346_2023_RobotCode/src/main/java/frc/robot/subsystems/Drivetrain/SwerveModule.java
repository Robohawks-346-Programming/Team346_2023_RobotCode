package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{
    CANSparkMax driveMotor, turnMotor;
    //TalonFX drivTalonFX, turnTalonFX;
 
    RelativeEncoder driveEncoder, turnEncoder;

    CANCoder turningCANCoder;

    double encoderOffset;

    SparkMaxPIDController driveController, turnController;

    public SwerveModule(
                        int driveMotorID,
                        int turnMotorID,
                        int CANCoderID,
                        double turnEncoderOffset) {
       
        encoderOffset = turnEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        turningCANCoder = new CANCoder(CANCoderID);
        turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turningCANCoder.setPosition(0);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.enableVoltageCompensation(Constants.MAX_VOLTAGE);
        turnMotor.enableVoltageCompensation(Constants.MAX_VOLTAGE);

        driveMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);

        driveEncoder.setVelocityConversionFactor(Constants.DRIVE_CONVERSION / 60);
        driveEncoder.setPositionConversionFactor(Constants.DRIVE_CONVERSION);

        turnEncoder.setPositionConversionFactor(360.0 / Constants.TURN_CONVERSION);
        
        driveController = driveMotor.getPIDController();
        turnController = turnMotor.getPIDController();

        driveController.setP(Constants.DRIVE_P);
        driveController.setI(Constants.DRIVE_I);
        driveController.setD(Constants.DRIVE_D);
        driveController.setFF(Constants.DRIVE_FF);

        turnController.setP(Constants.TURN_P);
        turnController.setI(Constants.TURN_I);
        turnController.setD(Constants.TURN_D);
        turnController.setFF(Constants.TURN_FF);

        driveMotor.burnFlash();
        turnMotor.burnFlash();
        
        //syncTurnEncoders(); Might need
    }

    public SwerveModuleState getState() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getPosition());
        return new SwerveModuleState(driveEncoder.getVelocity(), currentAngle);
    }

    public double deltaAdjustedAngle(double target, double current) {
        return((target - current + 180) % 360 + 360) %360 -180;
    }

    public Rotation2d adjustedAngle = new Rotation2d();
     
    public void setState(SwerveModuleState state) {
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getPosition());
        double delta = deltaAdjustedAngle(state.angle.getDegrees(), currentAngle.getDegrees());
        // double currentAngle = turnEncoder.getPosition();
        // double delta = deltaAdjustedAngle(state.angle.getDegrees(), currentAngle);
        double velocityInput = state.speedMetersPerSecond;
        double driveOutput = (state.speedMetersPerSecond) / 11.63659; // Testing showed we needed to divide by 11.63659, for some reason :( 

        if(Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + currentAngle.getDegrees());

        SmartDashboard.putNumber("Commanded Velocity", velocityInput);
        SmartDashboard.putNumber("Commanded Position", adjustedAngle.getDegrees());

        turnController.setReference(adjustedAngle.getDegrees(), ControlType.kPosition);
        driveController.setReference(driveOutput, ControlType.kVelocity, 0, Constants.DRIVE_FF * driveOutput);
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public void setOpenLoopState(SwerveModuleState state) {
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getPosition());
        double delta = deltaAdjustedAngle(state.angle.getDegrees(), currentAngle.getDegrees());
        double driveOutput = state.speedMetersPerSecond;

        if (Math.abs(delta) > 90) {
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }

        adjustedAngle = Rotation2d.fromDegrees(delta + currentAngle.getDegrees());

        SmartDashboard.putNumber("Commanded Velocity", driveOutput);
        SmartDashboard.putNumber("Commanded Position", adjustedAngle.getDegrees());

        turnController.setReference(adjustedAngle.getDegrees(), ControlType.kPosition);
        driveMotor.setVoltage(Constants.DRIVE_FF * driveOutput);
    }

    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }

    public void syncTurnEncoders() {
        turnEncoder.setPosition(turningCANCoder.getAbsolutePosition());
    }

    public void resetEncoders() {
        turnEncoder.setPosition(0.0);
        turningCANCoder.setPosition(0.0);
        turningCANCoder.configMagnetOffset(turningCANCoder.configGetMagnetOffset()- turningCANCoder.getAbsolutePosition());
    }

    public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveEncoder.getPosition());
    }

    public SwerveModulePosition getPosition() {
        double distance = getMetersDriven();
        Rotation2d angle = getTurnAngle();
        return new SwerveModulePosition(distance, angle);
    };
    

    private Rotation2d getTurnAngle() {
        return new Rotation2d(turnAngleRadians());
    }

    public double turnAngleRadians() {
        return encoderOffset + (turnEncoder.getPosition() * 2 * Math.PI);
    }
}