package frc.robot.commands.arm;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class DeliverLevel3 extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DeliverLevel3() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new GrabberIn(),
          new MoveArmLevel3(),
          new ExtendPneumatic1(),
          new ExtendPneumatic2(),
          new WaitCommand(2.5),
          new GrabberOut(),
          new RetractPneumatic1(),
          new RetractPneumatic2()
        )
      )
      
    );
  }



}
