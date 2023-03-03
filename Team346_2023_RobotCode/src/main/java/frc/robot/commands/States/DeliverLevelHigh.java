package frc.robot.commands.States;

import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Arm.RetractPneumatic1;
import frc.robot.commands.Arm.RetractPneumatic2;
import frc.robot.commands.Grabber.GrabberCloseManual;
import frc.robot.commands.Grabber.GrabberOpen;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class DeliverLevelHigh extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DeliverLevelHigh() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new GrabberOpen(),
          new WaitCommand(0.1),
          new RetractPneumatic1(),
          new RetractPneumatic2(),
          new WaitCommand(2),
          new GrabberCloseManual(),
          new MoveArm(Constants.HOME_ARM_ANGLE)
        )
      )
      
    );
  }



}
