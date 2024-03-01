// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandJoystick driver = new CommandJoystick(0);
  private final SendableChooser<Command> autoChooser;
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  /* Driver Buttons */
  private final Trigger zeroGyro = driver.button(2);
  private final Trigger robotCentric = driver.button(3);
  private final Trigger liftSolenoid = driver.button(6);
  private final Trigger blockSolenoid = driver.button(1);
  private final Trigger grabNote = driver.button(7);
  private final Trigger launchNote = driver.button(8); 

  /* Subsystems */
  private final Swerve m_swerve = new Swerve();
  private final Pneumatics m_pneumatics = new Pneumatics();
  private final Launcher m_launcher = new Launcher();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println(translationAxis);
    System.out.println(strafeAxis);
    System.out.println(rotationAxis);
    m_pneumatics.reverseLiftSolenoid();
    m_pneumatics.reverseBlockSolenoid();
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis-1)/3,
            () -> robotCentric.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    
    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // switches on controller labled from left to right: leftmost is number one, next is number two when it is closest to the joystick, and 3 when furthest, next is 4 when closest to joystick, 5 when furthest, last is 6 when closest. 
    // left button on bottom is 7, right button is 8
    // Power buttons, dials don't seem to do anything
    //left button pickup
    // right button launch
    // far right stick up-- activate hooks
    // right stick down-- hooks down
    // left stick-- blockers up blockers down
    // 5 -> inverse 2
    // 4 -> 3
    // 1 -> inverse 1
    // 0 -> 0
    zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    liftSolenoid.onTrue(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    liftSolenoid.onFalse(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    blockSolenoid.onTrue(new InstantCommand(() -> m_pneumatics.toggleBlockSolenoid()));
    blockSolenoid.onFalse(new InstantCommand(() -> m_pneumatics.toggleBlockSolenoid()));
    grabNote.onTrue(new InstantCommand(() -> m_launcher.setGrabMotors()));
    grabNote.onFalse(new InstantCommand(() -> m_launcher.stopGrabMotors()));
    launchNote.onTrue(new InstantCommand(() -> m_launcher.setLaunchMotors()));
    launchNote.onFalse(new InstantCommand(() -> m_launcher.stopLaunchMotors()));
    
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        2.0));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {

      Pose2d currentPose = m_swerve.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

}

