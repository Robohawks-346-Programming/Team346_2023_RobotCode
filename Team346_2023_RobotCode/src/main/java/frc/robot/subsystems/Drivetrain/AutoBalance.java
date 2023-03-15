package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class AutoBalance {
    private BuiltInAccelerometer mRioAccel;
    private int debounceCount;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private boolean onChargeStation = false;
    private boolean levelOn = false;
    private int levelCheck = 2;
    // 1 means need to move forward slowly
    // -1 meand need to move backward slowly
    // 2 is initliazation
    // 0 means level checked

    public AutoBalance() {
        mRioAccel = new BuiltInAccelerometer();
        debounceCount = 0;


        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = 6.0;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 0.2;


    }

    public double getPitch() {
        return Math.atan2((-mRioAccel.getX()),
                Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // Drive at fast speed until method below returns true
    public boolean getOnToChargeStation() {
        if (getTilt() > onChargeStationDegree) {
            debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
            debounceCount = 0;
            onChargeStation = true;
        }
        return onChargeStation;
    }

    // Drive at slow speed until level
    public boolean levelOnChargeStation() {
        if (getTilt() < levelDegree) {
            debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
            debounceCount = 0;
            levelOn = true;
        }
        return levelOn;
   }

   public int checkBalance() {
    if (Math.abs(getTilt()) <= levelDegree / 2) {
        debounceCount++;
    }
    if (debounceCount > secondsToTicks(debounceTime)) {
        debounceCount = 0;
        // Read comments under initilization
        levelCheck = 0;
    }
    if (getTilt() >= levelDegree) {
        // Read comments under initilization
        levelCheck = 1;
    } else if (getTilt() <= -levelDegree) {
        // Read comments under initilization
        levelCheck = -1;
    }
    return levelCheck;
   }
}