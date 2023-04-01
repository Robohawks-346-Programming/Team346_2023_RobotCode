package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static CANSparkMax intakeMotor;
    private boolean intakeValue;
    private DigitalInput laserBreak;
    
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        intakeValue = false;

        laserBreak = new DigitalInput(Constants.INTAKE_LASER_BREAK_PORT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Value", intakeValue);
        SmartDashboard.putBoolean("Intake laser", getLaserBreak());
    }
    // Extends Pneumatic part of the Intake
    public void extendIntake() {
        //intakeMotor.set(Constants.INTAKE_OUT_FAST_MOTOR_SPEED);
        intakeValue = true;
    }

    // Retracts Pneumatic part of the Intake
    public void retractIntake() {
        //intakeMotor.set(Constants.INTAKE_IN_MOTOR_SPEED);
        intakeValue = false;
    }

    // Runs Intake Motor
    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }
    
    // Stops Intake Motor when finished
    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public boolean getLaserBreak() {
        return !laserBreak.get();
    }

}
