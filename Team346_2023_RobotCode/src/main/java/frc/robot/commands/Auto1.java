package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Intake.RunIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;

public class Auto1 extends SequentialCommandGroup {

    public Auto1() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new RunIntakeOut()),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(new WaitCommand(6.1),
                        new DriveForTime(RobotContainer.drivetrain, -0.5, 0, 0.25)),
                    new ParallelDeadlineGroup(new WaitCommand(0.1),
                        new InstantCommand(RobotContainer.drivetrain::brake)),
                            new ParallelDeadlineGroup(new WaitCommand(2.5),
                                new RunIntakeIn())
        )));
    }
}

