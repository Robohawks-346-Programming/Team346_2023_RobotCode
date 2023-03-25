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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    
    private final SwerveDriveOdometry odometry; 
    private int debounceCount;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private boolean onChargeStation = false;
    private boolean levelOn = false;
    private int levelCheck = 2;
    private Compressor compressor;
    // 1 means need to move forward slowly
    // -1 meand need to move backward slowly
    // 2 is initliazation
    // 0 means level checked

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

        compressor = new Compressor(PneumaticsModuleType.REVPH);
        for( SwerveModule module : modules) {
            module.resetDistance();
        }

        driveConstants = new PIDConstants(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D);
        turnConstants = new PIDConstants(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

        debounceCount = 0;


        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = 6.0;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 0.5;
        onChargeStation = false;

    }
    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("theta", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Gyro Pitch()", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll()", gyro.getRoll());
        SmartDashboard.putNumber("Tilt", getTilt());
        SmartDashboard.putNumber("Wheel Encoder", getFrontLeftEncoder());
        SmartDashboard.putNumber("Back right encoder", backRight.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Back left encoder", backLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Y Acceleration", getAcceleration());
        //SmartDashboard.putNumber("Balance Value", checkBalance());
        //SmartDashboard.putBoolean("OnChargeStation", getOnToChargeStation());
        SmartDashboard.putNumber("Gyro Yaw", getYaw());

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

    public double getTilt() {
        double pitch = gyro.getPitch();
        double roll = gyro.getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // Drive at fast speed until method below returns true
    public boolean getOnToChargeStation() {
        if ((gyro.getPitch() < -9) && (gyro.getPitch() > -15)) {
            debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
            //System.out.println("true");
            debounceCount = 0;
            onChargeStation = true;
        }
        else {
            //System.out.println("false");
        }
        return onChargeStation;
    }

    // Drive at slow speed until level
    public boolean levelOnChargeStation() {
        if (getTilt() > levelDegree) {
            debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
            debounceCount = 0;
            levelOn = true;
        }
        return levelOn;
   }

   // Checks Balance on charge station
   public int checkBalance() {
    if (Math.abs(getTilt()) < levelDegree) {   //(Math.abs(getTilt()) <= levelDegree / 2)
        debounceCount++;
    }
    if (debounceCount > secondsToTicks(debounceTime)) {
        debounceCount = 0;
        // Read comments under initilization
        levelCheck = 0;
    }
    if (getTilt() >= levelDegree) {
        debounceCount = 0;
        // Read comments under initilization
        levelCheck = -1;
    } else if (getTilt() < -levelDegree) {
        debounceCount = 0;
        // Read comments under initilization
        levelCheck = 1;
    }
    return levelCheck;
   }

   public double getAcceleration() {
    return Math.abs(gyro.getWorldLinearAccelY());
   }

   public double getYaw() {
    return gyro.getYaw();
   }

   public void enableCompressor() {
    compressor.enableDigital();
  }

  public void disableCompressor() {
    compressor.disable();
  }

  public double getFrontLeftEncoder() {
    return Math.abs(frontLeft.getDistance());
  }

  public void resetFrontLeftDistance() {
    frontLeft.resetDistance();
  }
}