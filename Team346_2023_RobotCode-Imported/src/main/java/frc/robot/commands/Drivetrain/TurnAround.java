package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class TurnAround extends CommandBase {

  Drivetrain drivetrain;
  double x,y,theta;
  double currentHeading;
  int direction;

  public TurnAround(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentHeading = RobotContainer.drivetrain.getHeading().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentHeading >= -180) {
      direction = 1;
    }
    else {
      direction = -1;
    }
    //System.out.println(currentHeading);

    ChassisSpeeds velocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, direction, drivetrain.getHeading()) 
      : new ChassisSpeeds(0, 0, direction);
    drivetrain.drive(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentHeading > -180) {
      return (RobotContainer.drivetrain.getHeading().getDegrees() <= (currentHeading - 180));
    }
    else if(currentHeading < -180) {
      return (RobotContainer.drivetrain.getHeading().getDegrees() >= (currentHeading + 180));
    }
    else {
      return (RobotContainer.drivetrain.getHeading().getDegrees() <= (currentHeading - 179));
    }
  }
}