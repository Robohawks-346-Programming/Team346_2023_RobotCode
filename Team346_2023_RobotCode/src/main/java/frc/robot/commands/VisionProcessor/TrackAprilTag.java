package frc.robot.commands.VisionProcessor;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.AlignCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class TrackAprilTag extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TrackAprilTag() {
    if (RobotContainer.visionProcessor.isVisible() == true) {
      // Use addRequirements() here to declare subsystem dependencies.
      double xSubstation = RobotContainer.visionProcessor.distanceFromTargetX(Constants.HEIGHT_OF_SUBSTATION_APRIL_TAG);
      double zSubstation = RobotContainer.visionProcessor.distanceFromTargetZ(Constants.HEIGHT_OF_SUBSTATION_APRIL_TAG);
      Pose2d distanceSubstation = new Pose2d(xSubstation, zSubstation, null);

      double xGrid = RobotContainer.visionProcessor.distanceFromTargetX(Constants.HEIGHT_OF_SUBSTATION_APRIL_TAG);
      double zGrid = RobotContainer.visionProcessor.distanceFromTargetZ(Constants.HEIGHT_OF_SUBSTATION_APRIL_TAG);
      Pose2d distanceGrid = new Pose2d(xGrid, zGrid, null);

      double aprilTagID = RobotContainer.visionProcessor.getAprilTagID();

      if ((aprilTagID == 5.0) || (aprilTagID == 4)) {
        addCommands(
          new ParallelCommandGroup(
            new SequentialCommandGroup(
                new AlignCommand(distanceSubstation)
            )
          )
        
        );
      }
      else {
        addCommands(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
              new AlignCommand(distanceGrid)
          )
        )
      
      );
      }
    }
  }



}
