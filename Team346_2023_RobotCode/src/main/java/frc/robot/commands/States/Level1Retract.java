package frc.robot.commands.States;

import frc.robot.commands.Arm.MoveArmHome;
import frc.robot.commands.Arm.MoveArmHomeToLevel1;
import frc.robot.commands.Arm.RetractPneumatic1;
import frc.robot.commands.Grabber.GrabberClose;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
          new GrabberClose(),
          new MoveArmHomeToLevel1(),
          new RetractPneumatic1(),
          new MoveArmHome()
        )
      )
      
    );
  }



}
