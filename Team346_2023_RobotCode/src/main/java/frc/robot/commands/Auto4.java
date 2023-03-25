package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Drivetrain.TurnAround;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;

public class Auto4 extends SequentialCommandGroup {

    public Auto4() {
        addCommands(
            new SequentialCommandGroup(
                new Level3Config(),
                new WaitCommand(0.25),
                new ParallelDeadlineGroup(new WaitCommand(0.2), 
                    new DeliverFast()),
                new StartingConfig(),
                new ParallelDeadlineGroup (new WaitCommand(1.4), new DriveForTime(RobotContainer.drivetrain, 0, 0, 1)),
                new ParallelDeadlineGroup (new WaitCommand(2), new DriveForTime(RobotContainer.drivetrain, -0.8, 0, 0)),
                new ParallelDeadlineGroup(new WaitCommand(0.05), new DriveForTime(RobotContainer.drivetrain, 0, 0, 0.1))


            )
        );
    }

}

