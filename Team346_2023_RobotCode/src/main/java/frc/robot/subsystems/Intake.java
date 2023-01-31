package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    DoubleSolenoid intakeSolenoid;
    CANSparkMax intakeMotor;
    
    public Intake() {
        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_OUT_PNEUMATIC_ID, Constants.INTAKE_IN_PNEUMATIC_ID);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    }

    // Extends Pneumatic part of the Intake
    public void extendIntake() {
        intakeSolenoid.set(Value.kForward);
    }

    // Retracts Pneumatic part of the Intake
    public void retractIntake() {
        intakeSolenoid.set(Value.kReverse);
    }

    // Runs Intake Motor
    public void runIntake() {
        intakeMotor.set(Constants.INTAKE_MOTOR_SPEED);
    }
    
    // Stops Intake Motor when finished
    public void stopIntake() {
        intakeMotor.set(0.0);
    }

}
