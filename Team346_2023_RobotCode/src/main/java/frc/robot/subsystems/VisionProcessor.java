package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class VisionProcessor extends SubsystemBase {
    
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

    // updates the theta value of a variable to equal the data table value
    public void updateTheta() { 
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        xAngle = tx.getDouble(0.0);
        yAngle = ty.getDouble(0.0);
    }

    // changes pipeline - command
    public void changePipeline(int newPipeline) {
        pipeline.setInteger(newPipeline);
    }

    // check if target is visible - DON'T USE UPDATE THETA COMMAND UNLESS YOU ADD TV TO THAT METHOD
    public boolean isVisible() {
        tv = limelightTable.getEntry("tv");     // use network table to check if target is visible
        return (tv.getBoolean(false));   
    } 

    // check if centered with target
    public boolean isCentered() {
        if (isVisible()) {
            updateTheta();
            return (Math.abs(xAngle) <= Constants.END_ANGLE_THRESHOLD);
        }
        else {
            return false;
        }
    }

    // calculate distance from reflective tape level 2 - in degrees NOT RADIANS
    public double distanceFromTarget(double targetHeight) {
        updateTheta();
        return (targetHeight/(Math.tan(yAngle + Constants.CAMERA_ANGLE))); 
    }

    // check if at target distancen
    public boolean atTargetDistance(double targetHeight) {
        if (Math.abs(distanceFromTarget(targetHeight) - Constants.END_DISTANCE) <= Constants.END_DISTANCE_THRESHOLD) {
            return true;
        }
        else {
            return false;
        }
    }

}