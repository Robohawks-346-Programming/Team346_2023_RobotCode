package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.*;

public class AutoBalance extends CommandBase {
  Drivetrain m_drive;
  PIDController m_rollController;
  PIDController m_turnController;

  public AutoBalance(Drivetrain drivetrain) {
    m_drive = drivetrain;
    addRequirements(m_drive);
    m_rollController = new PIDController(.1, 0, 0);
    m_turnController = new PIDController(1, 0, 0);

  }

  @Override
  public void execute() {
    //tells drive base to drive in the opposite direction of the roll
    //uses field relative so it shouldn't care for if it is forward or backwards
    // if (Math.abs(m_drive.getPose().getRotation().getRadians() - Math.PI) > Units.degreesToRadians(4)) {
    //   m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    //       m_turnController.calculate(m_drive.getPose().getRotation().getRadians(), Math.PI),
    //       m_drive.getPose().getRotation()));
    // } else {
if (Math.abs(m_drive.getPitchDegrees()) < 2.5) {
      m_drive.brake();
    } else if (Math.abs(m_drive.getPitchRadians()) < Units.degreesToRadians(11)) {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          m_rollController.calculate(m_drive.getPitchRadians(), 0) / 5, 0, 0,
          m_drive.getPose().getRotation()));
    } else {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          -Math.signum(m_drive.getPitchRadians()) * .7, 0, 0,
          m_drive.getPose().getRotation()));
    }

    // }
    //tells drive base to drive in the opposite direction of the roll if it is forward
    // m_drive.drive(new ChassisSpeeds(m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.brake();
  }
}