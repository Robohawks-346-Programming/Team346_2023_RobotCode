package frc.robot.commands.VisionProcessor;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class TrackReflectiveTape extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TrackReflectiveTape() {
    if (RobotContainer.visionProcessor.isVisible()) {
      // Use addRequirements() here to declare subsystem dependencies.
      double x1 = RobotContainer.visionProcessor.distanceFromTargetX(Constants.HEIGHT_OF_TARGET);
      double z1 = RobotContainer.visionProcessor.distanceFromTargetZ(Constants.HEIGHT_OF_TARGET);
      Pose2d distance = new Pose2d(x1, z1, null);

        addCommands(
          new ParallelCommandGroup(
              new AlignCommand(distance)
          )
        );
    }
  }
}
