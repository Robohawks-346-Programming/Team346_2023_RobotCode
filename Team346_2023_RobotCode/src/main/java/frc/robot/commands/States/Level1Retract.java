package frc.robot.commands.States;

import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Grabber.Grab;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class Level1Retract extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Level1Retract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new Grab(),
          new WaitCommand(0.5),
          new MoveArm(Constants.HOME_ARM_ANGLE)
          
        )
      )
      
    );
  }



}