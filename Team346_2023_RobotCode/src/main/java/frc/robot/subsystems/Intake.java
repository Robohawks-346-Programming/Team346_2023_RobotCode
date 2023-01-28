package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class Intake {
    DoubleSolenoid intakeSolenoid;
    CANSparkMax intakeMotor;
    
    public Intake() {
        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GRABBER_OUT_PNEUMATIC_ID, Constants.GRABBER_IN_PNEUMATIC_ID);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    }

    public void extendIntake() {
        intakeSolenoid.set(Value.kForward);
    }

    public void retractIntake() {
        intakeSolenoid.set(Value.kReverse);
    }

    public void runIntake() {
        intakeMotor.set(Constants.INTAKE_MOTOR_SPEED);
    }
    
    public void stopIntake() {
        intakeMotor.set(0.0);
    }

}
