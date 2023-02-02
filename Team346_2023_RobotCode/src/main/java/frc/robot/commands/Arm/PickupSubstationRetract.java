package frc.robot.commands.arm;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class PickupSubstationRetract extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PickupSubstationRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new GrabberClose(),
          new WaitCommand(2),
          new MoveArmHome()
        )
      )
      
    );
  }



}