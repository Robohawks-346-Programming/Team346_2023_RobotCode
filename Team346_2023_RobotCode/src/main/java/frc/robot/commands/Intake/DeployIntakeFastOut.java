package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DeployIntakeFastOut extends SequentialCommandGroup {

    public DeployIntakeFastOut() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new RunIntakeFastOut(), new MoveIntake(Constants.INTAKE_OUT_POSITION)),
                new MoveIntake(Constants.INTAKE_IN_POSITION)
            )
        );
    }
}

