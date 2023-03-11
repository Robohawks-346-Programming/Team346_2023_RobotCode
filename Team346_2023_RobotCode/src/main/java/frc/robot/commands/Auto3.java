package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Auto3 extends SequentialCommandGroup {

    public Auto3() {
        // var group = PathPlanner.loadPathGroup(
        //     "3 cube + 1",
        //     new PathConstraints(2, 2));
    
        // return autoBuilder.fullAuto(group);
        // var group = PathPlanner.loadPathGroup("2 Cube New", 2, 2);
        // Command path1 = RobotContainer.autoBuilder.followPath(group.get(0));
        // Command path2 = RobotContainer.autoBuilder.followPath(group.get(1));
        
        // addCommands(
        //     new SequentialCommandGroup(
        //         path1,
        //         new InstantCommand(RobotContainer.drivetrain::brake)),
        //         path2,
        //         new InstantCommand(RobotContainer.drivetrain::brake));
    }

}

