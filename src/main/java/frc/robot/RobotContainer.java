// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RollingAverage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private Joystick driverJoystick; //Joysticks
  private Joystick operatorJoystick;  
  
  private JoystickButton intakeBalls; //buttons
  private JoystickButton alignDrivetrain;
  private JoystickButton expelBalls;

  private Drivetrain drivetrain;

  public RollingAverage throttle, wheel;

  private JoystickButton quickturn;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    driverJoystick = new Joystick(0);

    // intakeBalls = new JoystickButton(operatorJoystick, 0);
    // alignDrivetrain = new JoystickButton(operatorJoystick, 0);
    // expelBalls = new JoystickButton(operatorJoystick, 0);

    quickturn = new JoystickButton(driverJoystick, 5);
    drivetrain = new Drivetrain();
    throttle = new RollingAverage(50);
    wheel = new RollingAverage(15);
    
    configureButtonBindings();
    //SmartDashboard.putData("Run Flywheel", new SetRPM(shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // the :: syntax allows us to pass in methods of a class as variables so that the command can continuously access input values
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));

    // intakeBalls.whenPressed(new IntakeBalls(intake)).whenReleased(new StopIntaking(intake));
    // expelBalls.whenPressed(new ExpelBalls(storage));
    // alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new AlignIntakeToBall(drivetrain, false));
    /*
      Shoot: 4
      Intake: 5
      Expell Balls: <find>
      Climb: <find for operator>
      Turret Manual: ????
      Align To ball: 0
    */

  }

  /**
   * Gets the throttle input from the Driver Joystick throttle axis which is
   * used to move the robot forward and backwards
   * 
   * @return value from [-1, 1] that is used for input for the the robot forward or backwards movement
   */
  public double getThrottle() {
    throttle.updateValue(-driverJoystick.getRawAxis(5) * .7);
    return throttle.getAverage();
  }

  /**
   * Gets the wheel input from the Driver Joystick wheel axis which is used to turn the robot while
   * either driving or quickturning
   * 
   * @return value from [-1, 1] that is used for input for the robots turning movement
   */
  public double getWheel() {
    wheel.updateValue(driverJoystick.getRawAxis(0) * .8);
    return wheel.getAverage();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Trajectory tr = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(new Translation2d(0, 1), new Translation2d(0,2)), new Pose2d(0, 3, new Rotation2d()), new TrajectoryConfig(10, 2));//PathPlanner.loadPath("TryPath", 4, 4);\
    Trajectory tr = PathPlanner.loadPath("TryPath", 4, 4);
    return drivetrain.getRamseteCommand(tr);
  }
}
