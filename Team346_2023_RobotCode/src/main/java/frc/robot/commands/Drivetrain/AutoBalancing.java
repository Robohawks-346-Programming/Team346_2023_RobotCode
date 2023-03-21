package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class AutoBalancing  extends CommandBase {

  ChassisSpeeds positiveVelocity;
  ChassisSpeeds negativeVelocity;
  double x;

  public AutoBalancing() {
    x = Constants.MAX_MOVE_VELOCITY_SLOW;
    positiveVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(x, 0, 0, RobotContainer.drivetrain.getHeading()) 
    : new ChassisSpeeds(x, 0, 0);
    negativeVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(x, 0, 0, RobotContainer.drivetrain.getHeading()) 
    : new ChassisSpeeds(-x, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (RobotContainer.drivetrain.checkBalance() == -1) {
        ;//RobotContainer.drivetrain.drive(negativeVelocity, true);
      } else if (RobotContainer.drivetrain.checkBalance() == 1) {
        ;//RobotContainer.drivetrain.drive(positiveVelocity, true);
      } else {
        ;//RobotContainer.drivetrain.brake();
      }
    

    RobotContainer.drivetrain.brake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.drivetrain.checkBalance() == 0;
  }
}