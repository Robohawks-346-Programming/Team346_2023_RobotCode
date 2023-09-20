// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.CubeLowNoMove;
import frc.robot.commands.Autos.OneConeBalance;
import frc.robot.commands.Autos.OneConeCubeBlue;
import frc.robot.commands.Autos.OneConeCubeRed;
import frc.robot.commands.Autos.OneCubeBalance;
import frc.robot.commands.Autos.TwoCubeBlue;
import frc.robot.commands.Autos.TwoCubeConduitBlue;
import frc.robot.commands.Autos.TwoCubeConduitRed;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.JoystickDriveFast;
import frc.robot.commands.Drivetrain.JoystickDriveReverse;
import frc.robot.commands.Grabber.Grab;
import frc.robot.commands.Intake.DeployIntakeFastOut;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.DeployIntakeSlowOut;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.RunIntakeFastOut;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.DeliverLevel2;
import frc.robot.commands.States.Level2Config;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Arm arm = new Arm();
  public static final Grabber grabber = new Grabber();
  public static final LED led = new LED();
  public static final Intake intake = new Intake();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final VisionProcessor visionProcessor = new VisionProcessor();
  public final PS4Controller driverControl = new PS4Controller(Constants.DRIVER_CONTROLLER_PORT);
  public static final Joystick operatorControl = new Joystick(Constants.OPERATOR_CONTROLLER_PORT);
  // public static final TwoCubeOutBlue twoCubeOutOpenBlue = new TwoCubeOutBlue();
  // public static final TwoCubeOutRed twoCubeOutOpenRed = new TwoCubeOutRed();
  public static final TwoCubeBlue twoCubeBlue = new TwoCubeBlue();
  // public static final TwoCubeRed twoCubeRed = new TwoCubeRed();
  public static final OneConeCubeBlue oneConeCubeOpenBlue = new OneConeCubeBlue();
  public static final OneConeCubeRed oneConeCubeOpenRed = new OneConeCubeRed();
  public static final TwoCubeConduitBlue twoCubeConduitBlue = new TwoCubeConduitBlue();
  public static final TwoCubeConduitRed twoCubeConduitRed = new TwoCubeConduitRed();
  public static final OneConeBalance oneConeBalance = new OneConeBalance();
  public static final OneCubeBalance oneCubeBalance = new OneCubeBalance();
  public static final CubeLowNoMove cubeLowNoMove = new CubeLowNoMove();
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final JoystickButton BUTTON_1 = new JoystickButton(operatorControl, 1),
      BUTTON_2 = new JoystickButton(operatorControl, 2),
      BUTTON_3 = new JoystickButton(operatorControl, 3),
      BUTTON_4 = new JoystickButton(operatorControl, 4),
      BUTTON_5 = new JoystickButton(operatorControl, 5),
      BUTTON_6 = new JoystickButton(operatorControl, 6),
      BUTTON_7 = new JoystickButton(operatorControl, 7),
      BUTTON_8 = new JoystickButton(operatorControl, 8),
      BUTTON_9 = new JoystickButton(operatorControl, 9),
      BUTTON_10 = new JoystickButton(operatorControl, 10),
      BUTTON_11 = new JoystickButton(operatorControl, 11),
      BUTTON_12 = new JoystickButton(operatorControl, 12),
      BUTTON_13 = new JoystickButton(operatorControl, 13),
      BUTTON_14 = new JoystickButton(operatorControl, 14),
      BUTTON_15 = new JoystickButton(operatorControl, 15),
      BUTTON_16 = new JoystickButton(operatorControl, 16);

    public DoubleSupplier xAxis = () -> (-driverControl.getLeftY());
    public DoubleSupplier yAxis = () -> (-driverControl.getLeftX());
    public DoubleSupplier thetaAxis = () -> (-driverControl.getRightX());



  // PathPlannerTrajectory path = PathPlanner.loadPath("MoveOnly", new
  // PathConstraints(1, 1));
  // Command auto = RobotContainer.autoBuilder.fullAuto(path);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // SmartDashboard.putString("Auto Selector", "None");

    // Configure the trigger bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));
    configureAutoPaths();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    new JoystickButton(driverControl, Button.kOptions.value).onTrue(new InstantCommand(drivetrain::resetEncoders));
    new JoystickButton(driverControl, Button.kShare.value).onTrue(new InstantCommand(drivetrain::zeroHeading));
    new JoystickButton(driverControl, Button.kR2.value)
        .whileTrue(new JoystickDriveFast(drivetrain, xAxis, yAxis, thetaAxis));
    new JoystickButton(driverControl, Button.kL1.value).onTrue(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));
    new JoystickButton(driverControl, Button.kL2.value).whileTrue(new JoystickDriveReverse(drivetrain, xAxis, yAxis, thetaAxis));
    //new JoystickButton(driverControl, Button.kCross.value).onTrue(new InstantCommand(drivetrain::resetAbsoluteEncoders));

    //BUTTON_1.onTrue(new StartingConfig());
    BUTTON_1.whileTrue(new Grab());
    BUTTON_3.onTrue(new Level2Config());
    BUTTON_4.onTrue(new Level3Config());
    BUTTON_8.onTrue(new StartingConfig());
    BUTTON_9.whileTrue(new Deliver());
    BUTTON_10.whileTrue(new DeliverLevel2());
    //BUTTON_11.onTrue(new SubstationRetract());
    //BUTTON_12.onTrue(new SubstationConfig());
    BUTTON_13.whileTrue(new DeployIntakeIn());
    BUTTON_13.whileFalse(new MoveIntake(Constants.INTAKE_IN_POSITION));
    BUTTON_14.whileTrue(new DeployIntakeFastOut());
    BUTTON_15.whileTrue(new RunIntakeFastOut());
    BUTTON_16.whileTrue(new RunIntakeOut());

  }

  public void configureAutoPaths() {
    // autoChooser.addOption("2 Cube Out Open Blue", twoCubeOutOpenBlue);
    // autoChooser.addOption("2 Cube Out Open Red", twoCubeOutOpenRed);
    autoChooser.addOption("2 Cube Open Blue", twoCubeBlue);
    // autoChooser.addOption("2 Cube Open Red", twoCubeRed);
    autoChooser.addOption("1 Cone Cube Open Blue", oneConeCubeOpenBlue);
    autoChooser.addOption("1 Cone Cube Open Red", oneConeCubeOpenRed);
    autoChooser.addOption("2 Cube Conduit Blue", twoCubeConduitBlue);
    autoChooser.addOption("2 Cube Conduit Red", twoCubeConduitRed);
    autoChooser.addOption("1 Cone Balance", oneConeBalance);
    autoChooser.addOption("1 Cube Balance", oneCubeBalance);
    autoChooser.addOption("1 Cube Low No Move", cubeLowNoMove);

    SmartDashboard.putData("autoChooser", autoChooser);

  }

  public Command resetOdometry(List<PathPlannerTrajectory> pathGroup) {
      return new InstantCommand( () -> {
          PathPlannerState initialState = pathGroup.get(0).getInitialState();
          drivetrain.zeroHeading();
          drivetrain.setFieldToVehicle(new Rotation2d(0), drivetrain.getModulePositions(), new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
      });
  }
    // An example command will be run in autonomous
  public Command getAutonomousCommand() {
    HashMap<String, Command> eventMap = new HashMap<>();
    SwerveAutoBuilder builder = new SwerveAutoBuilder(
        drivetrain::getPose, // odometry position
        drivetrain::resetOdometry,
        new PIDConstants(0.05,0,0),
        new PIDConstants(1,0,0),
        drivetrain::drive, // this sets the motor powers
        eventMap,
        true,
        drivetrain);
        List<PathPlannerTrajectory> test = PathPlanner.loadPathGroup("Move", new PathConstraints(1, 1));
        Command path1 = builder.followPath(test.get(0));
        Command move = new SequentialCommandGroup(
          resetOdometry(test),
          new FollowPathWithEvents(path1, null, eventMap)
          //path1
        );
        autoChooser.addOption("move1", move);
        autoChooser.setDefaultOption("move1", move);
        configureAutoPaths();
        return autoChooser.getSelected();

    //     var group1 = PathPlanner.loadPathGroup("TwoCubeOpen", new PathConstraints(1, 1));
    //     Command path11 = builder.followPath(group1.get(0));
    //     Command path12 = builder.followPath(group1.get(1));
    //     Command twoCubeOpen = 
    //     new SequentialCommandGroup(path11, path12); 
    //     // new SequentialCommandGroup(
    //     //     new Level3Config(),
    //     //     new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //     //       new DeliverFast()),
    //     //     new ParallelRaceGroup(
    //     //       new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //     //         path11),
    //     //     new InstantCommand(drivetrain::brake),
    //     //     path12,
    //     //     new InstantCommand(drivetrain::brake),
    //     //     new ParallelDeadlineGroup(new WaitCommand(1), new RunIntakeOut()));

    //       var group2 = PathPlanner.loadPathGroup("ThreeCubeOpen", new PathConstraints(1, 1));
    //       Command path21 = builder.followPath(group2.get(0));
    //       Command path22 = builder.followPath(group2.get(1));
    //       Command path23 = builder.followPath(group2.get(2));
    //       Command path24 = builder.followPath(group2.get(3));
    //       Command path25 = builder.followPath(group2.get(4));
    //       Command threeCubeOpen = 
    //       new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //         new ParallelRaceGroup(
    //           new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //             path21),
    //         new InstantCommand(drivetrain::brake),
    //         path22,
    //         new InstantCommand(drivetrain::brake),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), new RunIntakeOut()),
    //         new ParallelRaceGroup(
    //           new DeployIntakeIn(),
    //           path23),
    //             new InstantCommand(drivetrain::brake),
    //             path24,
    //             new InstantCommand(drivetrain::brake),
    //             new ParallelDeadlineGroup(new WaitCommand(0.5), new RunIntakeOut()),
    //         new ParallelRaceGroup(
    //               new DeployIntakeIn(),
    //               path25)
    //         );

    //         var group3 = PathPlanner.loadPathGroup("TwoCubeOpen+Balance", new PathConstraints(2, 2));
    //         Command path31 = builder.followPath(group3.get(0));
    //         Command path32 = builder.followPath(group3.get(1));
    //         Command path33 = builder.followPath(group3.get(2));
    //         Command path34 = builder.followPath(group3.get(3));
    //         Command twoCubeOpenBalance = 
    //         new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //         new ParallelRaceGroup(
    //           new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //             path31),
    //         new InstantCommand(drivetrain::brake),
    //         path32,
    //         new InstantCommand(drivetrain::brake),
    //         new ParallelDeadlineGroup(new WaitCommand(1), new RunIntakeOut()),
    //         path33,
    //         path34,
    //         new AutoBalance());

    //         var group4 = PathPlanner.loadPathGroup("TwoCubeConduit", new PathConstraints(2, 2));
    //         Command path41 = builder.followPath(group4.get(0));
    //         Command path42 = builder.followPath(group4.get(1));
    //         Command twoCubeConduit =
    //         new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //         new ParallelRaceGroup(
    //           new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //             path41),
    //         new InstantCommand(drivetrain::brake),
    //         path42,
    //         new InstantCommand(drivetrain::brake),
    //         new ParallelDeadlineGroup(new WaitCommand(1), new RunIntakeOut()));

    //         var group5 = PathPlanner.loadPathGroup("ThreeCubeOpen", new PathConstraints(2, 3));
    //         Command path51 = builder.followPath(group5.get(0));
    //         Command path52 = builder.followPath(group5.get(1));
    //         Command path53 = builder.followPath(group5.get(2));
    //         Command path54 = builder.followPath(group5.get(3));
    //         Command path55 = builder.followPath(group5.get(4));
    //         Command threeCubeConduit = 
    //         new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //         new ParallelRaceGroup(
    //           new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //             path51),
    //         new InstantCommand(drivetrain::brake),
    //         path52,
    //         new InstantCommand(drivetrain::brake),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), new RunIntakeOut()),
    //         new ParallelRaceGroup(
    //           new DeployIntakeIn(),
    //           path53),
    //             new InstantCommand(drivetrain::brake),
    //             path54,
    //             new InstantCommand(drivetrain::brake),
    //             new ParallelDeadlineGroup(new WaitCommand(0.5), new RunIntakeOut()),
    //         new ParallelRaceGroup(
    //               new DeployIntakeIn(),
    //               path55)
    //         );

    //         var group6 = PathPlanner.loadPathGroup("OneCubeMiddle+Balance", 2, 3);
    //         Command path61 = builder.followPathGroup(group6);
    //         Command oneCubeMiddleBalance = 
    //         new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //           new StartingConfig(),
    //           path61,
    //           new AutoBalance());

              //  var group7 = PathPlanner.loadPath("Move", new PathConstraints(1, 1));
              //  Command auto11 = builder.followPath(group7);

            // autoChooser.addOption("2 Cube Open", twoCubeOpen);
            // autoChooser.addOption("3 Cube Open", threeCubeOpen);
            // autoChooser.addOption("2 Cube Open + Balance", twoCubeOpenBalance);
            // autoChooser.addOption("2 Cube Conduit", twoCubeConduit);
            // autoChooser.addOption("3 Cube Conduit", threeCubeConduit);
            // autoChooser.addOption("1 Cube Middle + Balance", oneCubeMiddleBalance);
            // autoChooser.setDefaultOption("2 Cube Open", twoCubeOpen);
  }
}
