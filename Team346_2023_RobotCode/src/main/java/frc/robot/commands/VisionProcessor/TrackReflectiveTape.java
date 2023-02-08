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
    if (RobotContainer.visionProcessor.isVisible() == true) {
      // Use addRequirements() here to declare subsystem dependencies.
      double x1 = RobotContainer.visionProcessor.distanceFromTargetX(Constants.HEIGHT_OF_TARGET);
      double z1 = RobotContainer.visionProcessor.distanceFromTargetZ(Constants.HEIGHT_OF_TARGET);
      Pose2d distance1 = new Pose2d(x1, z1, null);

      double x2 = RobotContainer.visionProcessor.distanceFromTargetX(Constants.HEIGHT_OF_TARGET);
      double z2 = RobotContainer.visionProcessor.distanceFromTargetZ(Constants.HEIGHT_OF_TARGET);
      Pose2d distance2 = new Pose2d(x2, z2, null);

      double aprilTagID = RobotContainer.visionProcessor.getAprilTagID();

      if (aprilTagID == 5.0) {
        addCommands(
          new ParallelCommandGroup(
              new AlignCommand(distance1),
              new AlignCommand(distance2)
          )
        );
      }
    }
  }
}
