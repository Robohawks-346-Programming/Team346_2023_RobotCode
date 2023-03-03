// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto1;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drivetrain.JoystickDrive;
import frc.robot.commands.Drivetrain.JoystickDriveFast;
import frc.robot.commands.Drivetrain.SyncEncoder;
import frc.robot.commands.Grabber.GrabberCloseManual;
import frc.robot.commands.Intake.DeployIntakeIn;
import frc.robot.commands.Intake.DeployIntakeSlowOut;
import frc.robot.commands.Intake.RunIntakeOut;
import frc.robot.commands.LED.LEDCone;
import frc.robot.commands.LED.LEDCube;
import frc.robot.commands.Intake.DeployIntakeFastOut;
import frc.robot.commands.States.DeliverLevelHigh;
import frc.robot.commands.States.Level1Config;
import frc.robot.commands.States.Level1Deliver;
import frc.robot.commands.States.Level1Retract;
import frc.robot.commands.States.Level2Config;
import frc.robot.commands.States.Level3Config;
import frc.robot.commands.States.StartingConfig;
import frc.robot.commands.States.SubstationConfig;
import frc.robot.commands.States.SubstationRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.VisionProcessor;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.commands.Arm.MoveArm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final Arm arm = new Arm();
  public static final Grabber grabber = new Grabber();
  public static final LED led = new LED();
  public static final Intake intake = new Intake();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final VisionProcessor visionProcessor = new VisionProcessor();
  public final PS4Controller driverControl = new PS4Controller(Constants.DRIVER_CONTROLLER_PORT);
  public static final Joystick operatorControl = new Joystick(Constants.OPERATOR_CONTROLLER_PORT);
  public static final Auto1 auto1 = new Auto1();

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

    static HashMap<String, Command> eventMap = new HashMap<>();

    public static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, // odometry position
      drivetrain::resetOdometry,
      Constants.DRIVE_KINEMATICS,
      drivetrain.getDriveConstants(),
      drivetrain.getTurnConstants(),
      drivetrain::setModuleState, // this sets the motor powers
      eventMap,
      true,
      drivetrain);
      
      // PathPlannerTrajectory path = PathPlanner.loadPath("MoveOnly", new PathConstraints(1, 1));   
      // Command auto = RobotContainer.autoBuilder.fullAuto(path);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new JoystickButton(driverControl, Button.kOptions.value).onTrue(new InstantCommand(drivetrain::resetEncoders));
    new JoystickButton(driverControl, Button.kR2.value).onTrue(new JoystickDriveFast(drivetrain, xAxis, yAxis, thetaAxis));
    new JoystickButton(driverControl, Button.kL2.value).onTrue(new JoystickDrive(drivetrain, xAxis, yAxis, thetaAxis));

    BUTTON_1.onTrue(new Level1Deliver());
    BUTTON_2.onTrue(new Level1Config());
    BUTTON_3.onTrue(new Level2Config());
    BUTTON_4.onTrue(new Level3Config());
    BUTTON_5.onTrue(new GrabberCloseManual());
    BUTTON_6.onTrue(new LEDCone());
    BUTTON_7.onTrue(new LEDCube());
    BUTTON_8.onTrue(new StartingConfig());
    BUTTON_9.onTrue(new Level1Retract());
    BUTTON_10.onTrue(new DeliverLevelHigh());
    BUTTON_11.onTrue(new SubstationRetract());
    BUTTON_12.onTrue(new SubstationConfig());
    BUTTON_13.whileTrue(new DeployIntakeIn());
    BUTTON_14.whileTrue(new DeployIntakeFastOut());
    BUTTON_15.whileTrue(new DeployIntakeSlowOut());
    BUTTON_16.whileTrue(new RunIntakeOut());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auto1;
  }
}
