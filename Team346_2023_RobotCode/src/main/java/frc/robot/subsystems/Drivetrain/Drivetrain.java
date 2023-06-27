package frc.robot.subsystems.Drivetrain;

import java.util.Optional;

import com.fasterxml.jackson.databind.node.BooleanNode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
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

    AHRS gyro = new AHRS();

    private final double[] lockAngles = new double[] { 45, 315, 45, 315 };

    public final SwerveDrivePoseEstimator poseEstimator; 

    private double lastFPGATimestamp;

    PIDConstants driveConstants, turnConstants;

    public Drivetrain(Pose2d startPose) {
        gyro.calibrate();; //Ask if this would work
        
        poseEstimator = new SwerveDrivePoseEstimator(Constants.DRIVE_KINEMATICS, getHeading(), getModulePositions(), startPose);
        
        for( SwerveModule module : modules) {
            module.resetDistance();
            module.syncTurnEncoders();
        }

        driveConstants = new PIDConstants(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
        turnConstants = new PIDConstants(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

        lastFPGATimestamp = Timer.getFPGATimestamp();
    }

    
    // @Override
    // public void periodic() {
    //     // poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    //     // if (lastFPGATimestamp < Timer.getFPGATimestamp()) {
    //     //     lastFPGATimestamp = Timer.getFPGATimestamp() + 1;
    //     //     for (SwerveModule module : modules) {
    //     //         module.syncTurnEncoders();
    //     //     }
    //     // }
    // }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        float rawYaw = gyro.getYaw();
        float calcYaw = rawYaw;
        if(0.0 > rawYaw) {
            calcYaw +=360.0;
        }
        return Rotation2d.fromDegrees(-calcYaw);
    }

    // public SwerveModulePosition[] getModulePositions() {
    //     SwerveModulePosition[] position = new SwerveModulePosition[4];

    //     for (int i2=0; i2<=3; i2++) {
    //         position[i2++] = modules[i2].getPosition(); 
    //     }

    //     return position;
    // }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleState() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i=0; i<=3; i++) {
            states[i++] = modules[i].getState(); 
        }

        return states;
    }

    public void brake() {
        for (int i = 0; i < modules.length; i++) {
          modules[i].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(lockAngles[i])));
        }
    }

    public void drive(ChassisSpeeds speeds, boolean normalize) {
        if (normalize){
        SwerveModuleState[] moduleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], modules[i].getStateAngle());
        }

        setModuleStates(moduleStates);
    }
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
          modules[i].setState(moduleStates[i]);
        }
    }

    public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
    }    

    public void resetEncoders() {
        for (SwerveModule module : modules) {
                module.resetEncoders();
        }
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    // @ Override
    // public void periodic() {
    //     if (RobotContainer.visionProcessor.getEstimatedPose() != null) {
    //         poseEstimator.addVisionMeasurement(RobotContainer.visionProcessor.getEstimatedPose(), lastFPGATimestamp);
    //     }
    // }
}
