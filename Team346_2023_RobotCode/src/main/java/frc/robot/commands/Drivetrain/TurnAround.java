package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class TurnAround extends CommandBase {

  Drivetrain drivetrain;
  double x,y,theta;
  double wantedHeading;

  public TurnAround(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wantedHeading = RobotContainer.drivetrain.getHeading().getDegrees() + 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds velocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1, drivetrain.getHeading()) 
      : new ChassisSpeeds(0, 0, 1);

    drivetrain.drive(velocity, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.drivetrain.getHeading().getDegrees() >= wantedHeading;
  }
}