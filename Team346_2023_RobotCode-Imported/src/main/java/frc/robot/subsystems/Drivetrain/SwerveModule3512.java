
package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;

public class SwerveModule3512 extends SubsystemBase {

   CANSparkMax driveMotor, turnMotor;

   RelativeEncoder driveEncoder, turnEncoder;

   CANCoder turningCANCoder;

   private Rotation2d lastAngle;

   double encoderOffset;

   SparkMaxPIDController driveController, turnController;

   private double adjustedSpeed;

   public SwerveModule3512 (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID) 
    {

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
        //driveController.setFF(Constants.DRIVE_FF);

        turnController.setP(Constants.TURN_P);
        turnController.setI(Constants.TURN_I);
        turnController.setD(Constants.TURN_D);
        turnController.setFF(Constants.TURN_FF);

        driveMotor.burnFlash();
        turnMotor.burnFlash();  
    }

    private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
        Constants.driveKS, Constants.driveKV, Constants.driveKA);

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
      }

      private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
          double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_MOVE_VELOCITY;
          driveMotor.set(percentOutput);
        } else {
          driveController.setReference(
              desiredState.speedMetersPerSecond,
              ControlType.kVelocity,
              0,
              feedforward.calculate(desiredState.speedMetersPerSecond));
        }
      }

      private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_MOVE_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;
    
        turnController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
      }

      private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnEncoder.getPosition());
      }
    
      public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(turningCANCoder.getAbsolutePosition());
      }
    
      public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
      }
}
