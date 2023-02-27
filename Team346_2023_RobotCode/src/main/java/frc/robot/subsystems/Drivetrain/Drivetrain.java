package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    
    private final SwerveDriveOdometry odometry; 

    SwerveModule frontLeft = new SwerveModule(
            Constants.FRONT_LEFT_DRIVE_ID,
            Constants.FRONT_LEFT_TURN_ID,
            Constants.FRONT_LEFT_ENCODER_ID,
            Constants.FRONT_LEFT_TURN_OFFSET
        );

    SwerveModule frontRight = new SwerveModule(
            Constants.FRONT_RIGHT_DRIVE_ID,
            Constants.FRONT_RIGHT_TURN_ID,
            Constants.FRONT_RIGHT_ENCODER_ID,
            Constants.FRONT_RIGHT_TURN_OFFSET
        ); 

    SwerveModule backLeft = new SwerveModule(
            Constants.BACK_LEFT_DRIVE_ID,
            Constants.BACK_LEFT_TURN_ID,
            Constants.BACK_LEFT_ENCODER_ID,
            Constants.BACK_LEFT_TURN_OFFSET
        );

    SwerveModule backRight = new SwerveModule(
            Constants.BACK_RIGHT_DRIVE_ID,
            Constants.BACK_RIGHT_TURN_ID,
            Constants.BACK_RIGHT_ENCODER_ID,
            Constants.BACK_RIGHT_TURN_OFFSET
        );

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    //ADIS16448_IMU gyro = new ADIS16448_IMU();
    AHRS gyro = new AHRS();

    PIDConstants driveConstants, turnConstants;

    public Drivetrain() {
        gyro.calibrate();; //Ask if this would work
        
        odometry = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS, getHeading(), getModulePositions());

        for( SwerveModule module : modules) {
            module.resetDistance();
        }

        driveConstants = new PIDConstants(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
        turnConstants = new PIDConstants(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);
    }

    public PIDConstants getDriveConstants() {
        driveConstants = new PIDConstants(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
        return driveConstants;
    }

    public PIDConstants getTurnConstants() {
        turnConstants = new PIDConstants(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);
        return turnConstants;
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(getHeading(), getModulePositions(), pose2d);
    }

    private void updateOdometry() {
        odometry.update(getHeading(), getModulePositions());
    }

    public void drive(ChassisSpeeds speed, boolean normalize) {
        if(speed.vxMetersPerSecond == 0 && speed.vyMetersPerSecond == 0 && speed.omegaRadiansPerSecond == 0) {
            brake();
            return;
        }

        SwerveModuleState[] swerveStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(speed);

        if(normalize) {
            normalDrive(swerveStates, speed);
        }
        setModuleState(swerveStates);
    }

    public void brake() {
        for (SwerveModule module : modules) {
            module.setState(new SwerveModuleState(0, module.getState().angle));
        }
    }

    public void normalDrive(SwerveModuleState[] desiredState, ChassisSpeeds speed) {
        double translationK = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) / Constants.MAX_MOVE_VELOCITY;
        double rotationK = Math.abs(speed.omegaRadiansPerSecond) / Constants.MAX_TURN_VELOCITY;
        double k = Math.max(translationK, rotationK);

        double currentMaxSpeed = 0.0;
        for (SwerveModuleState moduleState: desiredState) {
            currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        double scale = Math.min(k * Constants.MAX_VELOCITY / currentMaxSpeed, 1);
        for (SwerveModuleState moduleState : desiredState) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }

    public void setModuleState(SwerveModuleState[] states) {
        for (int i = 0; i<=3; i++) {
            modules[i].setState(states[i]);
        }
    }

    public void setOpenLoopState(SwerveModuleState[] desiredState) {
        for (int i = 0; i<=3; i++) {
            modules[i].setOpenLoopState(desiredState[i]);
        }
    }

    public SwerveModuleState[] getModuleState() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i=0; i<=3; i++) {
            states[i++] = modules[i].getState(); 
        }

        return states;
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public Rotation2d getHeading() {
        float rawYaw = gyro.getYaw();
        float calcYaw = rawYaw;
        if(0.0 > rawYaw) {
            calcYaw +=360.0;
        }
        return Rotation2d.fromDegrees(-calcYaw);
    }

    public void syncEncoders() {
        for(SwerveModule module : modules) {
            module.syncTurnEncoders();
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }
}