package frc.robot.commands.States;

import frc.robot.commands.Arm.MoveArmLevel2;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class Level2Config extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Level2Config() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new MoveArmLevel2()
        )
      )
      
    );
  }



}
