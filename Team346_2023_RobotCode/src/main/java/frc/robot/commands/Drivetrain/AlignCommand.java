package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class AlignCommand extends CommandBase {

    private PIDController xController, yController, thetaController;
    private Pose2d targetPose;

    public AlignCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        // say I want to use this at 3 meters
        // I want v = 1 m/s when d = 3 meters
        // estimate: d * kP = v, kP = v / d = 1 / 3
        double kTransP = 1/3;
        double kRotP = kTransP * (Constants.THEORETICAL_MAX_TURN_SPEED / Constants.THEORETICAL_MAX_MOVE_SPEED) * (180 / Math.PI);
        this.xController = new PIDController(kTransP, 0, 0);
        this.yController = new PIDController(kTransP, 0, 0);
        this.thetaController = new PIDController(kRotP, 0, 0);
        thetaController.enableContinuousInput(-180, 180);

    }

    @Override
    public void execute() {
        Pose2d currentPose = RobotContainer.drivetrain.getPose();

        double xSpeed = limitOutput(xController.calculate(currentPose.getX(), targetPose.getX()),
                0.25 * Constants.THEORETICAL_MAX_MOVE_SPEED);
        double ySpeed = limitOutput(yController.calculate(currentPose.getY(), targetPose.getY()),
                0.25 * Constants.THEORETICAL_MAX_MOVE_SPEED);
        double thetaSpeed = limitOutput(thetaController.calculate(currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees()), 0.25 * Constants.THEORETICAL_MAX_TURN_SPEED);

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        ChassisSpeeds percentSpeeds = new ChassisSpeeds(xSpeed / Constants.THEORETICAL_MAX_MOVE_SPEED,
                ySpeed / Constants.THEORETICAL_MAX_MOVE_SPEED, thetaSpeed / Constants.THEORETICAL_MAX_TURN_SPEED);

        RobotContainer.drivetrain.drive(speeds, percentSpeeds);
    }

    private double limitOutput(double input, double max) {
        return (Math.abs(input) > max) ? max : input;
    }
}
