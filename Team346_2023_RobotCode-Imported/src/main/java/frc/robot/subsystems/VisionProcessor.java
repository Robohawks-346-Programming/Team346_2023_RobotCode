package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class VisionProcessor extends SubsystemBase {


    private Optional<EstimatedRobotPose> curPose;
    private PhotonCamera cam;
    private Transform3d robotToCam;
    private PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout aprilTagFieldLayout;
    
    public VisionProcessor() {
        cam = new PhotonCamera("OV5647");
        robotToCam = new Transform3d(new Translation3d(0.1016, 0.0, 0.4191), new Rotation3d(0,0.436332,0)); //Cam mounted facing forward, 4 inches forward of center, 16.5 inches up from center.
            try {
                aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            } catch (IOException e) {
                e.printStackTrace();
            }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);
        cam.setPipelineIndex(0);
    }
    //Variables:
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelightTable"); // creates network table object
    NetworkTableEntry tx = limelightTable.getEntry("tx");              // defining tx using network table - used for xAngle
    NetworkTableEntry ty = limelightTable.getEntry("ty");              // defining ty using network table - used for yAngle
    NetworkTableEntry ta = limelightTable.getEntry("ta");              // defining ta using network table - used for area
    NetworkTableEntry tv = limelightTable.getEntry("tv");              // defining tv using network table -
    NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");  // defining pipeline using network table
    NetworkTableEntry ts = limelightTable.getEntry("ts");              // defining ts using network table

    private double xAngle = tx.getDouble(0.0);      // distance horizontaly using angles
    private double yAngle = ty.getDouble(0.0);      // distance verticly using angles

    // changes pipeline
    public void changePipeline(int newPipeline) {
        pipeline.setInteger(newPipeline);
    }

    // check if target is visible
    public boolean isVisible() {
        tv = limelightTable.getEntry("tv");     // use network table to check if target is visible
        return (tv.getBoolean(false));   
    } 

    // check if centered with target
    public boolean isCentered() {
        if (isVisible()) {
            xAngle = tx.getDouble(0.0);         // using network tables, check angle that target is at to camera
            return (Math.abs(xAngle) <= Constants.END_ANGLE_THRESHOLD);
        }
        else {
            return false;
        }
    }

    // calculate distance from target: TBD
    public double distanceFromTarget() {
        yAngle = ty.getDouble(0.0);
        return (Constants.HEIGHT_OF_TARGET/(Math.tan((yAngle + Constants.CAMERA_ANGLE)*(Math.PI/180)))); 
    }

    // check if at target distance
    public boolean atTargetDistance() {
        return (Math.abs(distanceFromTarget() - Constants.END_DISTANCE) <= Constants.END_DISTANCE_THRESHOLD);
    }
  
  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    if (cam.getPipelineIndex() == Constants.APRIL_TAG_PIPELINE) {
      if (photonPoseEstimator != null) {
        curPose = photonPoseEstimator.update();
        return curPose;
      } else {
        return Optional.empty();
      }
    }
    return Optional.empty();
  }


}
