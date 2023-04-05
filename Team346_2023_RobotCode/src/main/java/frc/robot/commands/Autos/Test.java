package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForEncoderDistance;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Drivetrain.TurnAround;
import frc.robot.commands.Intake.RunIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;

public class Test extends SequentialCommandGroup {

    public Test() {
        addCommands(
            new DriveForEncoderDistance(RobotContainer.drivetrain, 0.5, 0, 0, 2),
            new DriveForEncoderDistance(RobotContainer.drivetrain, 0, -0.5, 0, 2),
            new DriveForEncoderDistance(RobotContainer.drivetrain, -0.5, 0, 0, 2),
            new DriveForEncoderDistance(RobotContainer.drivetrain, 0, 0.5, 0, 2)
            );
    }

}


