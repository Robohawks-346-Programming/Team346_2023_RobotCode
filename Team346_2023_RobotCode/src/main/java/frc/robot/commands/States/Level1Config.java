package frc.robot.commands.States;

import frc.robot.Constants;
import frc.robot.commands.Arm.ExtendPneumatic1;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Arm.RetractPneumatic2;
import frc.robot.commands.Grabber.GrabberOpen;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class Level1Config extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Level1Config() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new MoveArm(Constants.LEVEL_1_ARM_ANGLE),
          new RetractPneumatic2(),
          new ExtendPneumatic1(),
          new GrabberOpen()
        )
      )
      
    );
  }



}
