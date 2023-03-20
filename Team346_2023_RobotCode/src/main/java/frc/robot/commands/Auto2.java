package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RunIntakeOut;

public class Auto2 extends SequentialCommandGroup {

    public Auto2() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new RunIntakeOut())
        ));
    }
}

