package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Drivetrain.TurnAround;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.RunIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;

public class TwoCubeRed extends SequentialCommandGroup {

    public TwoCubeRed() {
        addCommands(
            // new SequentialCommandGroup(
            //     new Level3Config(),
            //     new ParallelDeadlineGroup(new WaitCommand(1), new Deliver()),
            //     new StartingConfig(),
            //     new ParallelRaceGroup( 
            //         new DeployIntakeIn()),
            //         new ParallelDeadlineGroup(new WaitCommand(5),
            //             new DriveForTime(RobotContainer.drivetrain, 0.5, 0, 0)),
            //     new InstantCommand(RobotContainer.drivetrain::brake),
            //     new TurnAround(RobotContainer.drivetrain),
            //     new ParallelDeadlineGroup(new WaitCommand(5),
            //         new DriveForTime(RobotContainer.drivetrain, -0.5, 0, 0)),
            //     new RunIntakeOut()
            new SequentialCommandGroup(
                new Level3Config(),
                new WaitCommand(0.25),
                new ParallelDeadlineGroup(new WaitCommand(0.25), 
                    new DeliverFast()),
                new ParallelRaceGroup(
                    new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
                    new ParallelDeadlineGroup(new WaitCommand(5), 
                        new DriveForTime(RobotContainer.drivetrain, -0.5, 0.01, 0))),
                new InstantCommand(RobotContainer.drivetrain::brake),
                new ParallelCommandGroup(
                    new MoveIntake(Constants.INTAKE_IN_POSITION),
                    new ParallelDeadlineGroup (new WaitCommand(1.4), new DriveForTime(RobotContainer.drivetrain, 0, 0, 1))),
                new ParallelDeadlineGroup(new WaitCommand(5.1), 
                    new DriveForTime(RobotContainer.drivetrain, 0.5, 0.05, 0)),
                new InstantCommand(RobotContainer.drivetrain::brake),
                new ParallelDeadlineGroup(new WaitCommand(1), new RunIntakeOut())

            )
        );
    }

}

