package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.MoveArmHome;
import frc.robot.commands.Arm.RetractPneumatic1;
import frc.robot.commands.Arm.RetractPneumatic2;
import frc.robot.commands.Grabber.GrabberClose;

/** An example command that uses an example subsystem. */
public class StartingConfig extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartingConfig() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new RetractPneumatic1(),
          new RetractPneumatic2(),
          new GrabberClose(),
          new MoveArmHome()
        )
      )
      
    );
  }



}
