package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class DriveForEncoderDistance extends CommandBase {

  Drivetrain drivetrain;
  double x,y,theta;
  double distance;

  public DriveForEncoderDistance(Drivetrain drivetrain, double x, double y, double theta, double distance) {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.distance = distance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetFrontLeftDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double doubleX = Math.abs(x) < 0.05 ? 0 : x;
    double doubleY = Math.abs(y) < 0.05 ? 0 : y;
    double doubleTheta = Math.abs(theta) < 0.05 ? 0 : theta;

    double vx = doubleX * Constants.MAX_MOVE_VELOCITY;
    double vy = doubleY * Constants.MAX_MOVE_VELOCITY;
    double omega = doubleTheta * Constants.MAX_TURN_VELOCITY;

    ChassisSpeeds velocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading()) 
      : new ChassisSpeeds(vx, vy, omega);

    drivetrain.drive(velocity, true);
    boolean debugging = drivetrain.getFrontLeftMetersDriven() >= distance;
    SmartDashboard.putBoolean("debugging", debugging);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drivetrain.getFrontLeftMetersDriven() >= distance);
  }
}