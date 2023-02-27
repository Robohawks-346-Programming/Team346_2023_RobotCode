package frc.robot.commands.States;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Arm.RetractPneumatic1;
import frc.robot.commands.Arm.RetractPneumatic2;
import frc.robot.commands.Grabber.GrabberClose;
import frc.robot.commands.Grabber.GrabberCloseManual;

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
          new GrabberCloseManual(),
          new MoveArm(Constants.HOME_ARM_ANGLE)
        )
      )
      
    );
  }



}
