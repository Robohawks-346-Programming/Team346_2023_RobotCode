package frc.robot.commands.Autos;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Grabber.Grab;
import frc.robot.commands.Grabber.Release;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import frc.robot.Constants;

public class AutoFactory extends CommandBase {
    public static final PIDConstants linearPIDConstants = new PIDConstants(0.05, 0, 0);
    public static final PIDConstants angularPIDConstants = new PIDConstants(1, 0, 0);

    private final Drivetrain m_drive;
    private final Grabber m_grabber;
    private final Arm m_arm;
    private final Intake m_intake;

    private final Map<String, Command> m_eventMap;

    public AutoFactory(Drivetrain drivetrain, Grabber grabber, Arm arm, Intake intake) {
        m_drive = drivetrain;
        m_grabber = grabber;
        m_arm = arm;
        m_intake = intake;
    
        Command grab = new Grab();
        Command deliver = new Deliver();
        Command level3Config = new Level3Config();
        Command deliverFast = new DeliverFast();
        Command startingConfig = new StartingConfig();
        Command deployIntakeIn = new DeployIntakeIn();
        Command xBrake = new InstantCommand(drivetrain::brake);
        Command runIntakeOut = new RunIntakeOut();

        Command scoreHigh = Commands.sequence(
            level3Config,
            deliverFast,
            startingConfig  
        );

        m_eventMap = Map.ofEntries(
            Map.entry("deployIntakeIn", deployIntakeIn),
            Map.entry("runIntakeOut", runIntakeOut),
            Map.entry("scoreHigh", scoreHigh)
        );
        
    }

    public List<PathPlannerTrajectory> loadPathGroupByName(String name) {
        return PathPlanner.loadPathGroup(
            name,
            Constants.MAX_VELOCITY_AUTO,
            Constants.MAX_ACELLERATION_AUTO);
    }
    
      public CommandBase getAutoCommand(String name) {
        return getAutoCommand(loadPathGroupByName(name));
    }

    public Command resetOdometry(List<PathPlannerTrajectory> pathGroup) {
        return new InstantCommand( () -> {
            PathPlannerState initialState = pathGroup.get(0).getInitialState();
            m_drive.zeroHeading();
            m_drive.setFieldToVehicle(new Rotation2d(0), m_drive.getModulePositions(), new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
        });
    }

    private CommandBase getAutoCommand(List<PathPlannerTrajectory> paths) {
        // Create Auto builder
        BaseAutoBuilder builder = new SwerveAutoBuilder(
        m_drive::getPose, // odometry position
        m_drive::resetOdometry,
        new PIDConstants(0.05,0,0),
        new PIDConstants(1,0,0),
        m_drive::drive, // this sets the motor powers
        m_eventMap,
        true,
        m_drive);
        return Commands.sequence(
            new InstantCommand(m_drive::brake),
            resetOdometry(paths),
            builder.fullAuto(paths).andThen(new InstantCommand(m_drive::brake))
        );
    }
}
