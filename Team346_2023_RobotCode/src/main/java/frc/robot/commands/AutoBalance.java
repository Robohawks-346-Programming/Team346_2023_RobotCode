package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.AutoBalancing;
import frc.robot.commands.Drivetrain.AutoDriveUp;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Drivetrain.TurnAround;

public class AutoBalance extends SequentialCommandGroup {

    public AutoBalance() {
        addCommands(
            new SequentialCommandGroup(
                // new ParallelDeadlineGroup (new WaitCommand(2), new DriveForTime(RobotContainer.drivetrain, 0.7, 0, 0)),
                // new ParallelDeadlineGroup (new WaitCommand(3), new DriveForTime(RobotContainer.drivetrain, 0.3, 0, 0)),
                new AutoDriveUp()
                //new AutoBalancing(RobotContainer.drivetrain, 0.5, 0, 0)
                //new ParallelDeadlineGroup(new WaitCommand(2), 
                //new DriveForTime(RobotContainer.drivetrain, -0.25, 0, 0)),
                //new InstantCommand(RobotContainer.drivetrain::brake)
            )
        );
    }
}

