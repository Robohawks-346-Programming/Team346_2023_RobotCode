package frc.robot.subsystems.Drivetrain;

import java.util.Optional;

import com.fasterxml.jackson.databind.node.BooleanNode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
    SwerveModule frontLeft = new SwerveModule(
            Constants.FRONT_LEFT_DRIVE_ID,
            Constants.FRONT_LEFT_TURN_ID,
            Constants.FRONT_LEFT_ENCODER_ID
        );

        SwerveModule frontRight = new SwerveModule(
            Constants.FRONT_RIGHT_DRIVE_ID,
            Constants.FRONT_RIGHT_TURN_ID,
            Constants.FRONT_RIGHT_ENCODER_ID
        ); 

        SwerveModule backLeft = new SwerveModule(
            Constants.BACK_LEFT_DRIVE_ID,
            Constants.BACK_LEFT_TURN_ID,
            Constants.BACK_LEFT_ENCODER_ID
        );

        SwerveModule backRight = new SwerveModule(
            Constants.BACK_RIGHT_DRIVE_ID,
            Constants.BACK_RIGHT_TURN_ID,
            Constants.BACK_RIGHT_ENCODER_ID
        );

        SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    AHRS gyro = new AHRS();

    private final double[] lockAngles = new double[] { 45, 315, 45, 315 };

    public final SwerveDrivePoseEstimator poseEstimator; 

    private double lastFPGATimestamp;

    public Pose3d lastPose3d;

    PIDConstants driveConstants, turnConstants;

    public Drivetrain() {
        gyro.calibrate(); //Ask if this would work
        
        poseEstimator = new SwerveDrivePoseEstimator(Constants.DRIVE_KINEMATICS, gyro.getRotation2d(), getPositions(), new Pose2d());
        
        for( SwerveModule module : modules) {
            module.resetDistance();
            //module.syncTurnEncoders();
        }

        driveConstants = new PIDConstants(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
        turnConstants = new PIDConstants(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

        lastFPGATimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyro.getRotation2d(), getPositions());
        if (lastFPGATimestamp < Timer.getFPGATimestamp()) {
            lastFPGATimestamp = Timer.getFPGATimestamp() + 1;
            for (SwerveModule module : modules) {
                module.syncTurnEncoders();
            }
        }

        SmartDashboard.putNumber("Velocity Ouput", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Gyro Heading", gyro.getRotation2d().getDegrees());

        RobotContainer.visionProcessor.getEstimatedRobotPose().ifPresent(pose -> {
            lastPose3d = pose.estimatedPose;
            addVisionOdometryMeasurement(pose.estimatedPose, pose.timestampSeconds);
        });
    }

    public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
    }    

    public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_MOVE_VELOCITY);

    for (int i = 0; i < 4; i++) {
        modules[i].setDesiredState(swerveModuleStates[i], false);
    }
  }

  public void driveAuto(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    for (int i = 0; i < 4; i++) {
        modules[i].setDesiredState(moduleStates[i], false);
    }

    setModuleStates(moduleStates);
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_MOVE_VELOCITY);

    for (int i = 0; i < 4; i++) {
        modules[i].setDesiredState(desiredStates[i], false);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);;
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetEncoders() {
    for (SwerveModule module : modules) {
        module.resetEncoders();
    }
}

public void zeroHeading() {
    gyro.zeroYaw();
}

public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
    poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
}
}
