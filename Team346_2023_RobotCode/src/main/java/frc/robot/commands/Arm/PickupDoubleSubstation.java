package frc.robot.commands.Arm;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class PickupDoubleSubstation extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PickupDoubleSubstation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new RetractPneumatic1();
          new RetractPneumatic2();
          new MoveArmDoubleSubstation();
          new GrabberOut();
          new WaitCommand(2.5);
          new GrabberIn();
        )
      )
      
    );
  }



}
