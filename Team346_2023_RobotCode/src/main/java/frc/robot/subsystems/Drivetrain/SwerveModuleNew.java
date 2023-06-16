package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;

public class SwerveModuleNew extends SubsystemBase {

   CANSparkMax driveMotor, turnMotor;

   RelativeEncoder driveEncoder, turnEncoder;

   CANCoder turningCANCoder;

   double encoderOffset;

   SparkMaxPIDController driveController, turnController;

   public SwerveModuleNew (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID,
    double turnEncoderOffset) 
    {

        encoderOffset = turnEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        turningCANCoder = new CANCoder(turningCANCoderID);
        turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turningCANCoder.configSensorDirection(false);

        driveEncoder.setVelocityConversionFactor(Constants.DRIVE_CONVERSION / 60);
        driveEncoder.setPositionConversionFactor(Constants.DRIVE_CONVERSION);

        turnEncoder.setPositionConversionFactor(360.0 / Constants.TURN_CONVERSION);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.enableVoltageCompensation(Constants.MAX_VOLTAGE);
        turnMotor.enableVoltageCompensation(Constants.MAX_VOLTAGE);

        driveMotor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(Constants.TURN_CURRENT_LIMIT);

        driveController = driveMotor.getPIDController();
        turnController = turnMotor.getPIDController();

        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMinInput(-180);
        turnController.setPositionPIDWrappingMaxInput(180);

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
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getStateAngle());
    }

    public SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteRotation());
    }

    public Rotation2d getStateAngle() {
        double stateAngle = Units.degreesToRadians(turnEncoder.getPosition());
        return new Rotation2d(MathUtil.angleModulus(stateAngle));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromDegrees(turningCANCoder.getAbsolutePosition());
    }

    public Rotation2d adjustedAngle = new Rotation2d();

    public void setState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        driveController.setReference(driveOutput, ControlType.kVelocity, 0, Constants.DRIVE_FF * driveOutput);
    }

    public double adjustedAngle(double wantedAngle, double currentAngle) {
        return ((wantedAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }

    public void syncTurnEncoders() {
        turnEncoder.setPosition(turningCANCoder.getAbsolutePosition());
    }

    public void resetEncoders() {
        turnEncoder.setPosition(turningCANCoder.getAbsolutePosition()- encoderOffset);

        //turningCANCoder.setPosition(0);
        //turningCANCoder.configMagnetOffset(turningCANCoder.configGetMagnetOffset()- turningCANCoder.getAbsolutePosition());
    }
}
