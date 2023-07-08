package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DeployIntakeSlowOut extends SequentialCommandGroup {

    public DeployIntakeSlowOut() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new RunIntakeSlowOut(), new MoveIntake(Constants.INTAKE_OUT_POSITION)),
                new MoveIntake(Constants.INTAKE_IN_POSITION)
            )
        );
    }
}

