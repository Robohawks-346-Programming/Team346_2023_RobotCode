package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ExtendPneumatic1;
import frc.robot.commands.Arm.ExtendPneumatic2;
import frc.robot.commands.Arm.MoveArmLevel3;

/** An example command that uses an example subsystem. */
public class Level3Config extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Level3Config() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new MoveArmLevel3(),
          new ParallelCommandGroup(
            new ExtendPneumatic1(),
            new ExtendPneumatic2()
          )
        )
      )
      
    );
  }



}
