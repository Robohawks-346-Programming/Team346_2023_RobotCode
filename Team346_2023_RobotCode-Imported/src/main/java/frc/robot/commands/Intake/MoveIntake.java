// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  double position;
  double currentRev;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveIntake(double position) {
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  currentRev = RobotContainer.intake.getRotationEncoder();
  RobotContainer.intake.setRotationMotorToCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    RobotContainer.intake.moveIntake(position, currentRev);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stopRotationMotor();
    if (position == Constants.INTAKE_IN_POSITION){
    RobotContainer.intake.setRotationMotorToBrake();
    } else {
      RobotContainer.intake.setRotationMotorToCoast();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intake.isAtPosition(position);
  }
}
