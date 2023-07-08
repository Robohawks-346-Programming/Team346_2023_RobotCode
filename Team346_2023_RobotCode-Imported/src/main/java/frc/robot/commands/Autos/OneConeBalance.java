package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Intake.RunIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverCone;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;

public class OneConeBalance extends SequentialCommandGroup {

    public OneConeBalance() {
        addCommands(
            new SequentialCommandGroup(
                new Level3Config(),
                new WaitCommand(0.25),
                new ParallelDeadlineGroup(new WaitCommand(0.2), 
                    new DeliverCone()),
                new StartingConfig(),
                new ParallelDeadlineGroup (new WaitCommand(2), new DriveForTime(RobotContainer.drivetrain, -1, 0, 0)),
                new ParallelDeadlineGroup(new WaitCommand(0.05), new DriveForTime(RobotContainer.drivetrain, 0, 0, 0.1))


            )
        );
    }

}

