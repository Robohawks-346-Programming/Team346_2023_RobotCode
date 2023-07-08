package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DeployIntakeInPID extends SequentialCommandGroup {

    public DeployIntakeInPID() {
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(new RunIntakeIn(), new MoveIntakePID(Constants.INTAKE_OUT_POSITION)),
                new MoveIntakePID(Constants.INTAKE_IN_POSITION)
            )
        );
    }
}

