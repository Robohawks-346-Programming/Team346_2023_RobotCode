package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveForTime;
import frc.robot.commands.Drivetrain.JoystickDriveAutoDriveUp;
import frc.robot.commands.Drivetrain.JoystickDriveSlowAutoBalance;
import frc.robot.commands.Drivetrain.JoystickDriveSlowAutoCheckBalance;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;

public class AutoBalance extends SequentialCommandGroup {

    public AutoBalance() {
        addCommands(
            new SequentialCommandGroup(
                new JoystickDriveAutoDriveUp(null, 0, 0.2, 0),
                new JoystickDriveSlowAutoBalance(null, 0, 0.1, 0),
                new JoystickDriveSlowAutoCheckBalance(null, 0, 0.05, 0)
            )
        );
    }
}

