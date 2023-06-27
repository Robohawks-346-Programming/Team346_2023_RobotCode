package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
public class VisionProcessor extends SubsystemBase {

    public VisionProcessor() {
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

    public NetworkTableEntry getEstimatedPose() {
        return limelightTable.getEntry("camerapose_robotspace");
    }

}
