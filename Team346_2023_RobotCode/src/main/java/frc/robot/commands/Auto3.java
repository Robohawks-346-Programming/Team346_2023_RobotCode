package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;

public class Auto3 extends SequentialCommandGroup {

    public Auto3() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new RunIntakeOut()),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(new WaitCommand(5.9),
                        new DriveForTime(RobotContainer.drivetrain, -0.5, 0, 0)),
                    new ParallelDeadlineGroup(new WaitCommand(0.1),
                        new InstantCommand(RobotContainer.drivetrain::brake))
        )));
    }
}

