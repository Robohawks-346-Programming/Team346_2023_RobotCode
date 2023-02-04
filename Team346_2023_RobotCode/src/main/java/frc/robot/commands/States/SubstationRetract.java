package frc.robot.commands.States;

import frc.robot.commands.Arm.MoveArmHome;
import frc.robot.commands.Grabber.GrabberClose;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class SubstationRetract extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public double x;
  public SubstationRetract() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      CommandScheduler.getInstance().
    // if (x = SubstationConfig) {
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new GrabberClose(),
          new WaitCommand(2),
          new MoveArmHome()
        )
      // }
      // else if (x = Level1Config) {
      //   new ParallelCommandGroup(
      //   new SequentialCommandGroup(
      //     new GrabberClose(),
      //     new MoveArmHomeToLevel1(),
      //     new RetractPneumatic1(),
      //     new MoveArmHome()
      //   )
      // )
      // }
      // else {
      // new ParallelCommandGroup(
        // new SequentialCommandGroup (
        // )
      // )
      // }
        )
      )
      )
      
    );
  }



}
