// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.Auto3;
import frc.robot.commands.Auto4;
import frc.robot.commands.Auto5;
import frc.robot.commands.Auto6;
import frc.robot.commands.Auto7;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.JoystickDriveFast;
import frc.robot.commands.Drivetrain.JoystickDriveReverse;
import frc.robot.commands.Grabber.Grab;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.DeployIntakeSlowOut;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.LED.LEDCone;
import frc.robot.commands.LED.LEDCube;
import frc.robot.commands.Intake.DeployIntakeFastOut;
import frc.robot.commands.States.Deliver;
import frc.robot.commands.States.DeliverFast;
import frc.robot.commands.States.Level2Config;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.VisionProcessor;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public static final Auto1 auto1 = new Auto1();
  public static final Auto2 auto2 = new Auto2();
  public static final Auto3 auto3 = new Auto3();
  //public static final Auto4 auto4 = new Auto4();
  public static final Auto5 auto5 = new Auto5();
  public static final Auto6 auto6 = new Auto6();
  public static final Auto7 auto7 = new Auto7();
  public static final AutoBalance autoBalance = new AutoBalance();
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

  public static HashMap<String, Command> eventMap = new HashMap<>();

  public static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, // odometry position
      drivetrain::resetOdometry,
      new PIDConstants(0.01,0,0),
      new PIDConstants(1,0,0),
      speeds -> drivetrain.drive(speeds, true), // this sets the motor powers
      eventMap,
      drivetrain);

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
    configureAutoPaths();

    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));
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
    new JoystickButton(driverControl, Button.kR2.value).whileTrue(new InstantCommand(drivetrain::disableCompressor));
    new JoystickButton(driverControl, Button.kR2.value).whileFalse(new InstantCommand(drivetrain::enableCompressor));
    new JoystickButton(driverControl, Button.kL1.value).onTrue(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));
    new JoystickButton(driverControl, Button.kL2.value).whileTrue(new JoystickDriveReverse(drivetrain, xAxis, yAxis, thetaAxis));

    //BUTTON_1.onTrue(new StartingConfig());
    BUTTON_2.whileTrue(new Grab());
    BUTTON_3.onTrue(new Level2Config());
    BUTTON_4.onTrue(new Level3Config());
    BUTTON_6.whileTrue(new LEDCone());
    BUTTON_7.whileTrue(new LEDCube());
    BUTTON_8.onTrue(new StartingConfig());
    BUTTON_9.whileTrue(new Deliver());
    BUTTON_10.whileTrue(new DeliverFast());
    //BUTTON_11.onTrue(new SubstationRetract());
    //BUTTON_12.onTrue(new SubstationConfig());
    BUTTON_13.whileTrue(new DeployIntakeIn());
    BUTTON_14.whileTrue(new DeployIntakeFastOut());
    BUTTON_15.whileTrue(new DeployIntakeSlowOut());
    BUTTON_16.whileTrue(new RunIntakeOut());

  }

  public void configureAutoPaths() {
    // autoChooser.addOption("1 Cube Out", auto1);
    // autoChooser.addOption("1 Cube No Move", auto2);
    // autoChooser.setDefaultOption("1 Cube Out", auto1);
    var group = PathPlanner.loadPathGroup("TwoCubePart1", 1, 0.5);
    Command path1 = RobotContainer.autoBuilder.followPath(group.get(0));
    Command auto4 = new SequentialCommandGroup(path1, new InstantCommand(drivetrain::brake));
    // Command auto4 = 
    //     new SequentialCommandGroup(
    //         new Level3Config(),
    //         new ParallelDeadlineGroup(new WaitCommand(0.5), 
    //           new DeliverFast()),
    //         new ParallelRaceGroup(
    //           new SequentialCommandGroup(new StartingConfig(), new DeployIntakeIn()),
    //             path1),
    //         new InstantCommand(drivetrain::brake),
    //         path2,
    //         new InstantCommand(drivetrain::brake),
    //         new ParallelDeadlineGroup(new WaitCommand(1), new RunIntakeOut()));
    // autoChooser.addOption("2 Cube Left", auto3);

    autoChooser.setDefaultOption("Path Planner", auto4);
    autoChooser.addOption("2 Cube Blue", auto3);
    autoChooser.addOption("2 Cube Red", auto5);
    //autoChooser.addOption("Auto Balance", autoBalance);
    //autoChooser.addOption("AutoBalanceNew", auto4);
    autoChooser.addOption("2 Cube Blue Conduit", auto6);
    autoChooser.addOption("2 Cube Blue New", auto7);
    // autoChooser.setDefaultOption("2 Cube Blue", auto3);
    SmartDashboard.putData("autoChooser", autoChooser);
  }
    // An example command will be run in autonomous
  public Command getAutonomousCommand() {    
    return autoChooser.getSelected();
  }
}
