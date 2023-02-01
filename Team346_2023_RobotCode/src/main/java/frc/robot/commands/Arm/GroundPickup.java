package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class GroundPickup extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveOnly() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new MoveArmLevel1();
          new RetractPneumatic2();
          new ExtendPneumatic1();
          new GrabberOut();
          new WaitCommand(2.5);
          new GrabberIn();
          new MoveArmHomeToLevel1();
          new RetractPneumatic1();
        )
      )
      
    );
  }



}
