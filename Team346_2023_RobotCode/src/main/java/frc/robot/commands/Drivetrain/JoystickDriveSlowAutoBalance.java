package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class JoystickDriveSlowAutoBalance  extends CommandBase {

  Drivetrain drivetrain;
  double x;
double y;
double theta;

  public JoystickDriveSlowAutoBalance(Drivetrain drivetrain, double x, double y, double theta) {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.theta = theta;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double doubleX = Math.abs(x) < 0.05 ? 0 : x;
    double doubleY = Math.abs(y) < 0.05 ? 0 : y;
    double doubleTheta = Math.abs(theta) < 0.05 ? 0 : theta;

    double vx = doubleX * Constants.MAX_MOVE_VELOCITY_SLOW;
    double vy = doubleY * Constants.MAX_MOVE_VELOCITY_SLOW;
    double omega = doubleTheta * Constants.MAX_TURN_VELOCITY_SLOW;

    ChassisSpeeds velocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading()) 
      : new ChassisSpeeds(vx, vy, omega);

    drivetrain.drive(velocity, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.drivetrain.levelOnChargeStation();
  }
}