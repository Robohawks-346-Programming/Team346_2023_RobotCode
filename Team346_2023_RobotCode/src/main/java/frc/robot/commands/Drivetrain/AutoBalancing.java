package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class AutoBalancing  extends CommandBase {

  ChassisSpeeds positiveVelocity;
  ChassisSpeeds negativeVelocity;
  //double x;
  int balance;
  Drivetrain drivetrain;
  double x,y,theta;

  public AutoBalancing(Drivetrain drivetrain, double x, double y, double theta) {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.theta = theta;
    addRequirements(drivetrain);
    //x = Constants.MAX_MOVE_VELOCITY_SLOW;
    // positiveVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(x, 0, 0, RobotContainer.drivetrain.getHeading()) 
    // : new ChassisSpeeds(x, 0, 0);
    // negativeVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(-x, 0, 0, RobotContainer.drivetrain.getHeading()) 
    // : new ChassisSpeeds(-x, 0, 0);
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

    ChassisSpeeds positiveVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, RobotContainer.drivetrain.getHeading()) 
      : new ChassisSpeeds(vx, vy, omega);
    ChassisSpeeds negativeVelocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(-vx, vy, omega, RobotContainer.drivetrain.getHeading()) 
      : new ChassisSpeeds(-vx, vy, omega);

      RobotContainer.drivetrain.drive(positiveVelocity, true);
      // if(RobotContainer.drivetrain.checkBalance() == 1) {
      //   RobotContainer.drivetrain.drive(positiveVelocity, true);
  
      // }
      // else if (RobotContainer.drivetrain.checkBalance() == -1){
      //   RobotContainer.drivetrain.drive(negativeVelocity, true);
      // }
      // else {
      //   RobotContainer.drivetrain.brake();
      // }
    // balance = RobotContainer.drivetrain.checkBalance();
    //   if (balance == -1) {
    //     System.out.println("backward");
    //     RobotContainer.drivetrain.drive(negativeVelocity, true);
    //   } else if (balance == 1) {
    //     System.out.println("forward");
    //     RobotContainer.drivetrain.drive(positiveVelocity, true);
    //   } else {
    //     //RobotContainer.drivetrain.brake();
    //   }
     }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("end");
    RobotContainer.drivetrain.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return RobotContainer.drivetrain.checkBalance() == 0;
    return RobotContainer.drivetrain.getAcceleration() > 0.5;
  }
}