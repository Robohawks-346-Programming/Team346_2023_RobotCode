// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Arm;

// import frc.robot.Constants;
// import frc.robot.RobotContainer;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// /** An example command that uses an example subsystem. */
// public class MoveArmHomeToLevel1 extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public MoveArmHomeToLevel1() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     RobotContainer.arm.armBrakeOff();
//   }

//   Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     RobotContainer.arm.moveArm(Constants.ROTATE_UP_LEVEL_1_ARM_ANGLE);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.arm.stopRotationMotor();
//     RobotContainer.arm.armBrakeOn();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return RobotContainer.arm.isAtPosition(Constants.ROTATE_UP_LEVEL_1_ARM_ANGLE);
//   }
// }
